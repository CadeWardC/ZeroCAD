#![forbid(unsafe_code)]
//! STEP AP242 (ISO 10303-21) B-Rep Reader.

use std::collections::HashMap;
use std::fs;
use std::io;

use openrcad_foundation::{Ax2, Ax22d, Ax3, Dir, Dir2d, Pnt, Pnt2d};
use openrcad_geom::{BSplineCurve, BSplineSurface, Curve, GeomCurve, GeomSurface, Surface};
use openrcad_geom2d::{BSplineCurve2d, Circle2d, Curve2d, Ellipse2d, GeomCurve2d, Line2d};
use openrcad_topo::{
    arena::{BRep, EdgeData, FaceData, LoopData, OrientedEdge, ShellData, SolidData, VertexData},
    orientation::Orientation,
    PcurveData, Solid, SurfacePeriodicity,
};

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Ref(u32),
    Ident(String),
    StringVal(String),
    Enum(String),
    Number(f64),
    Int(i64),
    Equal,
    LParen,
    RParen,
    Comma,
    Semicolon,
    Dollar,
    Asterisk,
}

#[derive(Clone, Debug, PartialEq)]
pub enum StepValue {
    Integer(i64),
    Real(f64),
    String(String),
    Enum(String),
    Ref(u32),
    List(Vec<StepValue>),
    Omitted,
    Typed(String, Box<StepValue>),
}

#[derive(Clone, Debug, PartialEq)]
pub enum StepEntity {
    Simple { name: String, args: Vec<StepValue> },
    Complex(Vec<(String, Vec<StepValue>)>),
}

fn strip_comments(input: &str) -> String {
    let mut out = String::with_capacity(input.len());
    let mut chars = input.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '/' && chars.peek() == Some(&'*') {
            chars.next(); // consume '*'
            while let Some(c2) = chars.next() {
                if c2 == '*' && chars.peek() == Some(&'/') {
                    chars.next(); // consume '/'
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

fn tokenize(input: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();

    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
            continue;
        }

        match c {
            '=' => {
                chars.next();
                tokens.push(Token::Equal);
            }
            '(' => {
                chars.next();
                tokens.push(Token::LParen);
            }
            ')' => {
                chars.next();
                tokens.push(Token::RParen);
            }
            ',' => {
                chars.next();
                tokens.push(Token::Comma);
            }
            ';' => {
                chars.next();
                tokens.push(Token::Semicolon);
            }
            '$' => {
                chars.next();
                tokens.push(Token::Dollar);
            }
            '*' => {
                chars.next();
                tokens.push(Token::Asterisk);
            }
            '#' => {
                chars.next();
                let mut num_str = String::new();
                while let Some(&nc) = chars.peek() {
                    if nc.is_ascii_digit() {
                        num_str.push(nc);
                        chars.next();
                    } else {
                        break;
                    }
                }
                let id = num_str.parse::<u32>().map_err(|e| e.to_string())?;
                tokens.push(Token::Ref(id));
            }
            '\'' => {
                chars.next();
                let mut s = String::new();
                while let Some(&nc) = chars.peek() {
                    if nc == '\'' {
                        chars.next();
                        if chars.peek() == Some(&'\'') {
                            s.push('\'');
                            chars.next();
                        } else {
                            break;
                        }
                    } else {
                        s.push(nc);
                        chars.next();
                    }
                }
                tokens.push(Token::StringVal(s));
            }
            '.' => {
                chars.next();
                if let Some(&nc) = chars.peek() {
                    if nc.is_ascii_digit() {
                        let mut num_str = String::from(".");
                        while let Some(&nc2) = chars.peek() {
                            if nc2.is_ascii_digit()
                                || nc2 == '-'
                                || nc2 == '+'
                                || nc2 == 'e'
                                || nc2 == 'E'
                            {
                                num_str.push(nc2);
                                chars.next();
                            } else {
                                break;
                            }
                        }
                        let f = num_str.parse::<f64>().map_err(|e| e.to_string())?;
                        tokens.push(Token::Number(f));
                        continue;
                    }
                }

                let mut e_str = String::new();
                while let Some(&nc) = chars.peek() {
                    if nc == '.' {
                        chars.next();
                        break;
                    } else {
                        e_str.push(nc);
                        chars.next();
                    }
                }
                tokens.push(Token::Enum(e_str));
            }
            _ => {
                if c.is_ascii_alphabetic() || c == '_' {
                    let mut ident = String::new();
                    while let Some(&nc) = chars.peek() {
                        if nc.is_ascii_alphanumeric() || nc == '_' {
                            ident.push(nc);
                            chars.next();
                        } else {
                            break;
                        }
                    }
                    tokens.push(Token::Ident(ident));
                } else if c.is_ascii_digit() || c == '-' || c == '+' {
                    let mut num_str = String::new();
                    while let Some(&nc) = chars.peek() {
                        if nc.is_ascii_digit()
                            || nc == '.'
                            || nc == '-'
                            || nc == '+'
                            || nc == 'e'
                            || nc == 'E'
                        {
                            num_str.push(nc);
                            chars.next();
                        } else {
                            break;
                        }
                    }
                    if let Ok(i) = num_str.parse::<i64>() {
                        tokens.push(Token::Int(i));
                    } else if let Ok(f) = num_str.parse::<f64>() {
                        tokens.push(Token::Number(f));
                    } else {
                        return Err(format!("Invalid number format: {}", num_str));
                    }
                } else {
                    return Err(format!("Unexpected character: {}", c));
                }
            }
        }
    }

    Ok(tokens)
}

struct Parser<'a> {
    tokens: &'a [Token],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(tokens: &'a [Token]) -> Self {
        Self { tokens, pos: 0 }
    }

    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.pos)
    }

    fn next(&mut self) -> Option<&Token> {
        let t = self.tokens.get(self.pos);
        if t.is_some() {
            self.pos += 1;
        }
        t
    }

    fn consume(&mut self, expected: Token) -> Result<(), String> {
        match self.next() {
            Some(t) if *t == expected => Ok(()),
            Some(t) => Err(format!("Expected {:?}, found {:?}", expected, t)),
            None => Err(format!("Expected {:?}, found EOF", expected)),
        }
    }

    fn parse_value(&mut self) -> Result<StepValue, String> {
        match self.peek() {
            Some(Token::Int(i)) => {
                let val = StepValue::Integer(*i);
                self.pos += 1;
                Ok(val)
            }
            Some(Token::Number(f)) => {
                let val = StepValue::Real(*f);
                self.pos += 1;
                Ok(val)
            }
            Some(Token::StringVal(s)) => {
                let val = StepValue::String(s.clone());
                self.pos += 1;
                Ok(val)
            }
            Some(Token::Enum(e)) => {
                let val = StepValue::Enum(e.clone());
                self.pos += 1;
                Ok(val)
            }
            Some(Token::Ref(r)) => {
                let val = StepValue::Ref(*r);
                self.pos += 1;
                Ok(val)
            }
            Some(Token::Dollar) => {
                self.pos += 1;
                Ok(StepValue::Omitted)
            }
            Some(Token::Asterisk) => {
                self.pos += 1;
                Ok(StepValue::Omitted)
            }
            Some(Token::LParen) => {
                self.pos += 1;
                let mut list = Vec::new();
                if self.peek() == Some(&Token::RParen) {
                    self.pos += 1;
                    return Ok(StepValue::List(list));
                }
                loop {
                    list.push(self.parse_value()?);
                    match self.peek() {
                        Some(Token::Comma) => {
                            self.pos += 1;
                        }
                        Some(Token::RParen) => {
                            self.pos += 1;
                            break;
                        }
                        Some(t) => return Err(format!("Expected ',' or ')', found {:?}", t)),
                        None => return Err("Unexpected EOF in list".to_string()),
                    }
                }
                Ok(StepValue::List(list))
            }
            Some(Token::Ident(name)) => {
                let name = name.clone();
                self.pos += 1;
                self.consume(Token::LParen)?;
                let inner = self.parse_value()?;
                self.consume(Token::RParen)?;
                Ok(StepValue::Typed(name, Box::new(inner)))
            }
            Some(t) => Err(format!("Unexpected token in value: {:?}", t)),
            None => Err("Unexpected EOF".to_string()),
        }
    }

    fn parse_typed_args(&mut self) -> Result<(String, Vec<StepValue>), String> {
        let name = match self.next() {
            Some(Token::Ident(n)) => n.clone(),
            Some(t) => return Err(format!("Expected Ident, found {:?}", t)),
            None => return Err("Expected Ident, found EOF".to_string()),
        };
        self.consume(Token::LParen)?;
        let mut args = Vec::new();
        if self.peek() == Some(&Token::RParen) {
            self.pos += 1;
            return Ok((name, args));
        }
        loop {
            args.push(self.parse_value()?);
            match self.peek() {
                Some(Token::Comma) => {
                    self.pos += 1;
                }
                Some(Token::RParen) => {
                    self.pos += 1;
                    break;
                }
                Some(t) => return Err(format!("Expected ',' or ')', found {:?}", t)),
                None => return Err("Unexpected EOF in args".to_string()),
            }
        }
        Ok((name, args))
    }

    fn parse_entity(&mut self) -> Result<(u32, StepEntity), String> {
        let id = match self.next() {
            Some(Token::Ref(id)) => *id,
            Some(t) => return Err(format!("Expected Ref, found {:?}", t)),
            None => return Err("Expected Ref, found EOF".to_string()),
        };
        self.consume(Token::Equal)?;

        let entity = match self.peek() {
            Some(Token::LParen) => {
                self.pos += 1;
                let mut parts = Vec::new();
                while self.peek() != Some(&Token::RParen) {
                    parts.push(self.parse_typed_args()?);
                }
                self.pos += 1;
                StepEntity::Complex(parts)
            }
            _ => {
                let (name, args) = self.parse_typed_args()?;
                StepEntity::Simple { name, args }
            }
        };
        self.consume(Token::Semicolon)?;
        Ok((id, entity))
    }
}

fn parse_point(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Pnt, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;
    match ent {
        StepEntity::Simple { name, args } if name == "CARTESIAN_POINT" => {
            if args.len() >= 2 {
                if let StepValue::List(coords) = &args[1] {
                    if coords.len() >= 3 {
                        let x = match coords[0] {
                            StepValue::Real(f) => f,
                            StepValue::Integer(i) => i as f64,
                            _ => 0.0,
                        };
                        let y = match coords[1] {
                            StepValue::Real(f) => f,
                            StepValue::Integer(i) => i as f64,
                            _ => 0.0,
                        };
                        let z = match coords[2] {
                            StepValue::Real(f) => f,
                            StepValue::Integer(i) => i as f64,
                            _ => 0.0,
                        };
                        return Ok(Pnt::new(x, y, z));
                    }
                }
            }
            Err("Invalid CARTESIAN_POINT arguments".to_string())
        }
        _ => Err(format!(
            "Expected CARTESIAN_POINT at #{}, found {:?}",
            id, ent
        )),
    }
}

fn parse_dir(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Dir, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;
    match ent {
        StepEntity::Simple { name, args } if name == "DIRECTION" => {
            if args.len() >= 2 {
                if let StepValue::List(coords) = &args[1] {
                    if coords.len() >= 3 {
                        let x = match coords[0] {
                            StepValue::Real(f) => f,
                            StepValue::Integer(i) => i as f64,
                            _ => 0.0,
                        };
                        let y = match coords[1] {
                            StepValue::Real(f) => f,
                            StepValue::Integer(i) => i as f64,
                            _ => 0.0,
                        };
                        let z = match coords[2] {
                            StepValue::Real(f) => f,
                            StepValue::Integer(i) => i as f64,
                            _ => 0.0,
                        };
                        return Ok(Dir::new(x, y, z));
                    }
                }
            }
            Err("Invalid DIRECTION arguments".to_string())
        }
        _ => Err(format!("Expected DIRECTION at #{}, found {:?}", id, ent)),
    }
}

fn parse_vector(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<(Dir, f64), String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;
    match ent {
        StepEntity::Simple { name, args } if name == "VECTOR" => {
            if args.len() >= 3 {
                let dir_id = match args[1] {
                    StepValue::Ref(r) => r,
                    _ => return Err("Invalid VECTOR direction reference".to_string()),
                };
                let mag = match args[2] {
                    StepValue::Real(f) => f,
                    StepValue::Integer(i) => i as f64,
                    _ => 0.0,
                };
                let d = parse_dir(dir_id, entities)?;
                return Ok((d, mag));
            }
            Err("Invalid VECTOR arguments".to_string())
        }
        _ => Err(format!("Expected VECTOR at #{}, found {:?}", id, ent)),
    }
}

fn parse_axis2(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Ax3, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;
    match ent {
        StepEntity::Simple { name, args } if name == "AXIS2_PLACEMENT_3D" => {
            if args.len() >= 2 {
                let loc_ref = match args[1] {
                    StepValue::Ref(r) => r,
                    _ => return Err("Invalid AXIS2_PLACEMENT_3D location".to_string()),
                };
                let loc = parse_point(loc_ref, entities)?;

                let main_dir = if args.len() >= 3 {
                    match args[2] {
                        StepValue::Ref(r) => parse_dir(r, entities)?,
                        _ => Dir::new(0.0, 0.0, 1.0),
                    }
                } else {
                    Dir::new(0.0, 0.0, 1.0)
                };

                let x_dir = if args.len() >= 4 {
                    match args[3] {
                        StepValue::Ref(r) => Some(parse_dir(r, entities)?),
                        _ => None,
                    }
                } else {
                    None
                };

                if let Some(x) = x_dir {
                    let dot = main_dir.dot(&x);
                    let perp_x = if dot.abs() > 1e-6 {
                        let vx = openrcad_foundation::Vec::from_dir(x);
                        let vmain = openrcad_foundation::Vec::from_dir(main_dir);
                        let proj = vx - vmain.multiplied(dot);
                        Dir::from_vec(&proj).unwrap_or_else(|| {
                            // fallback pick X dir if normalizes to zero
                            let ax2 = Ax2::new(loc, main_dir);
                            ax2.x_direction()
                        })
                    } else {
                        x
                    };
                    let y = main_dir.cross(&perp_x);
                    return Ok(Ax3::new_full(loc, main_dir, perp_x, y));
                } else {
                    return Ok(Ax3::from(Ax2::new(loc, main_dir)));
                }
            }
            Err("Invalid AXIS2_PLACEMENT_3D arguments".to_string())
        }
        _ => Err(format!(
            "Expected AXIS2_PLACEMENT_3D at #{}, found {:?}",
            id, ent
        )),
    }
}

fn step_number(value: &StepValue) -> Option<f64> {
    match value {
        StepValue::Real(value) => Some(*value),
        StepValue::Integer(value) => Some(*value as f64),
        _ => None,
    }
}

fn parse_point2d(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Pnt2d, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("2D point entity #{id} not found"))?;
    match ent {
        StepEntity::Simple { name, args } if name == "CARTESIAN_POINT" => {
            let coordinates = match args.get(1) {
                Some(StepValue::List(values)) if values.len() >= 2 => values,
                _ => return Err(format!("invalid 2D CARTESIAN_POINT #{id}")),
            };
            let x = step_number(&coordinates[0])
                .ok_or_else(|| format!("invalid X coordinate in 2D point #{id}"))?;
            let y = step_number(&coordinates[1])
                .ok_or_else(|| format!("invalid Y coordinate in 2D point #{id}"))?;
            Ok(Pnt2d::new(x, y))
        }
        _ => Err(format!("expected 2D CARTESIAN_POINT at #{id}")),
    }
}

fn parse_dir2d(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Dir2d, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("2D direction entity #{id} not found"))?;
    match ent {
        StepEntity::Simple { name, args } if name == "DIRECTION" => {
            let coordinates = match args.get(1) {
                Some(StepValue::List(values)) if values.len() >= 2 => values,
                _ => return Err(format!("invalid 2D DIRECTION #{id}")),
            };
            let x = step_number(&coordinates[0])
                .ok_or_else(|| format!("invalid X coordinate in 2D direction #{id}"))?;
            let y = step_number(&coordinates[1])
                .ok_or_else(|| format!("invalid Y coordinate in 2D direction #{id}"))?;
            Ok(Dir2d::new(x, y))
        }
        _ => Err(format!("expected 2D DIRECTION at #{id}")),
    }
}

fn parse_vector2d(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Dir2d, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("2D vector entity #{id} not found"))?;
    match ent {
        StepEntity::Simple { name, args } if name == "VECTOR" => match args.get(1) {
            Some(StepValue::Ref(direction)) => parse_dir2d(*direction, entities),
            _ => Err(format!("invalid 2D VECTOR #{id}")),
        },
        _ => Err(format!("expected 2D VECTOR at #{id}")),
    }
}

fn parse_axis22d(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<Ax22d, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("2D axis entity #{id} not found"))?;
    match ent {
        StepEntity::Simple { name, args } if name == "AXIS2_PLACEMENT_2D" => {
            let location = match args.get(1) {
                Some(StepValue::Ref(point)) => parse_point2d(*point, entities)?,
                _ => return Err(format!("invalid 2D axis location #{id}")),
            };
            let direction = match args.get(2) {
                Some(StepValue::Ref(direction)) => parse_dir2d(*direction, entities)?,
                _ => Dir2d::dx(),
            };
            Ok(Ax22d::new(location, direction))
        }
        _ => Err(format!("expected AXIS2_PLACEMENT_2D at #{id}")),
    }
}

fn parse_bspline_curve2d(
    degree: usize,
    poles_values: &[StepValue],
    multiplicity_values: &[StepValue],
    knot_values: &[StepValue],
    weight_values: Option<&[StepValue]>,
    entities: &HashMap<u32, StepEntity>,
) -> Result<GeomCurve2d, String> {
    let poles = poles_values
        .iter()
        .map(|value| match value {
            StepValue::Ref(id) => parse_point2d(*id, entities),
            _ => Err("invalid 2D B-spline pole".to_string()),
        })
        .collect::<Result<Vec<_>, _>>()?;
    let multiplicities = multiplicity_values
        .iter()
        .map(|value| match value {
            StepValue::Integer(value) if *value > 0 => Ok(*value as usize),
            _ => Err("invalid 2D B-spline multiplicity".to_string()),
        })
        .collect::<Result<Vec<_>, _>>()?;
    let knots = knot_values
        .iter()
        .map(|value| step_number(value).ok_or_else(|| "invalid 2D B-spline knot".to_string()))
        .collect::<Result<Vec<_>, _>>()?;
    let weights = weight_values
        .map(|values| {
            values
                .iter()
                .map(|value| {
                    step_number(value).ok_or_else(|| "invalid 2D B-spline weight".to_string())
                })
                .collect::<Result<Vec<_>, _>>()
        })
        .transpose()?;
    if degree == 0
        || degree > poles.len()
        || poles.is_empty()
        || knots.len() != multiplicities.len()
        || multiplicities.iter().sum::<usize>() != poles.len() + degree + 1
        || weights
            .as_ref()
            .is_some_and(|values| values.len() != poles.len() || values.iter().any(|w| *w <= 0.0))
    {
        return Err("inconsistent 2D B-spline data".to_string());
    }
    Ok(GeomCurve2d::bspline(BSplineCurve2d::new(
        degree,
        poles,
        weights,
        knots,
        multiplicities,
    )))
}

fn parse_curve2d(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<GeomCurve2d, String> {
    let entity = entities
        .get(&id)
        .ok_or_else(|| format!("2D curve entity #{id} not found"))?;

    if let StepEntity::Complex(parts) = entity {
        let base = parts.iter().find(|(name, _)| name == "B_SPLINE_CURVE");
        let knots = parts
            .iter()
            .find(|(name, _)| name == "B_SPLINE_CURVE_WITH_KNOTS");
        let rational = parts
            .iter()
            .find(|(name, _)| name == "RATIONAL_B_SPLINE_CURVE");
        if let (Some((_, base)), Some((_, knots))) = (base, knots) {
            let degree = match base.first() {
                Some(StepValue::Integer(value)) if *value > 0 => *value as usize,
                _ => return Err(format!("invalid 2D B-spline degree #{id}")),
            };
            let poles = match base.get(1) {
                Some(StepValue::List(values)) => values.as_slice(),
                _ => return Err(format!("invalid 2D B-spline poles #{id}")),
            };
            let multiplicities = match knots.first() {
                Some(StepValue::List(values)) => values.as_slice(),
                _ => return Err(format!("invalid 2D B-spline multiplicities #{id}")),
            };
            let knot_values = match knots.get(1) {
                Some(StepValue::List(values)) => values.as_slice(),
                _ => return Err(format!("invalid 2D B-spline knots #{id}")),
            };
            let weights = rational.and_then(|(_, args)| match args.first() {
                Some(StepValue::List(values)) => Some(values.as_slice()),
                _ => None,
            });
            return parse_bspline_curve2d(
                degree,
                poles,
                multiplicities,
                knot_values,
                weights,
                entities,
            );
        }
    }

    match entity {
        StepEntity::Simple { name, args } => match name.as_str() {
            "LINE" => {
                let point = match args.get(1) {
                    Some(StepValue::Ref(id)) => parse_point2d(*id, entities)?,
                    _ => return Err(format!("invalid 2D LINE location #{id}")),
                };
                let direction = match args.get(2) {
                    Some(StepValue::Ref(id)) => parse_vector2d(*id, entities)?,
                    _ => return Err(format!("invalid 2D LINE direction #{id}")),
                };
                Ok(GeomCurve2d::line(Line2d::from_point_dir(point, direction)))
            }
            "CIRCLE" => {
                let axis = match args.get(1) {
                    Some(StepValue::Ref(id)) => parse_axis22d(*id, entities)?,
                    _ => return Err(format!("invalid 2D CIRCLE axis #{id}")),
                };
                let radius = args
                    .get(2)
                    .and_then(step_number)
                    .ok_or_else(|| format!("invalid 2D CIRCLE radius #{id}"))?;
                Ok(GeomCurve2d::circle(Circle2d::new(axis, radius)))
            }
            "ELLIPSE" => {
                let axis = match args.get(1) {
                    Some(StepValue::Ref(id)) => parse_axis22d(*id, entities)?,
                    _ => return Err(format!("invalid 2D ELLIPSE axis #{id}")),
                };
                let major = args
                    .get(2)
                    .and_then(step_number)
                    .ok_or_else(|| format!("invalid 2D ELLIPSE major radius #{id}"))?;
                let minor = args
                    .get(3)
                    .and_then(step_number)
                    .ok_or_else(|| format!("invalid 2D ELLIPSE minor radius #{id}"))?;
                Ok(GeomCurve2d::ellipse(Ellipse2d::new(axis, major, minor)))
            }
            "B_SPLINE_CURVE_WITH_KNOTS" => {
                let degree = match args.get(1) {
                    Some(StepValue::Integer(value)) if *value > 0 => *value as usize,
                    _ => return Err(format!("invalid 2D B-spline degree #{id}")),
                };
                let poles = match args.get(2) {
                    Some(StepValue::List(values)) => values.as_slice(),
                    _ => return Err(format!("invalid 2D B-spline poles #{id}")),
                };
                let multiplicities = match args.get(6) {
                    Some(StepValue::List(values)) => values.as_slice(),
                    _ => return Err(format!("invalid 2D B-spline multiplicities #{id}")),
                };
                let knots = match args.get(7) {
                    Some(StepValue::List(values)) => values.as_slice(),
                    _ => return Err(format!("invalid 2D B-spline knots #{id}")),
                };
                parse_bspline_curve2d(degree, poles, multiplicities, knots, None, entities)
            }
            _ => Err(format!("unsupported 2D curve type {name}")),
        },
        _ => Err(format!("unsupported 2D curve entity #{id}")),
    }
}

fn trim_parameter(value: &StepValue) -> Option<f64> {
    match value {
        StepValue::Real(value) => Some(*value),
        StepValue::Integer(value) => Some(*value as f64),
        StepValue::Typed(name, value) if name == "PARAMETER_VALUE" => trim_parameter(value),
        StepValue::List(values) => values.iter().find_map(trim_parameter),
        _ => None,
    }
}

fn parse_curve2d_representation(
    id: u32,
    entities: &HashMap<u32, StepEntity>,
) -> Result<(GeomCurve2d, Option<(f64, f64)>), String> {
    let Some(StepEntity::Simple { name, args }) = entities.get(&id) else {
        return parse_curve2d(id, entities).map(|curve| (curve, None));
    };
    if name != "TRIMMED_CURVE" {
        return parse_curve2d(id, entities).map(|curve| (curve, None));
    }

    let basis = match args.get(1) {
        Some(StepValue::Ref(curve)) => *curve,
        _ => return Err(format!("invalid TRIMMED_CURVE basis #{id}")),
    };
    let trim1 = args
        .get(2)
        .and_then(trim_parameter)
        .ok_or_else(|| format!("invalid first TRIMMED_CURVE parameter #{id}"))?;
    let trim2 = args
        .get(3)
        .and_then(trim_parameter)
        .ok_or_else(|| format!("invalid last TRIMMED_CURVE parameter #{id}"))?;
    let same_sense = matches!(args.get(4), Some(StepValue::Enum(value)) if value == "T");
    let curve = parse_curve2d(basis, entities)?;
    Ok((
        curve,
        Some(if same_sense {
            (trim1, trim2)
        } else {
            (trim2, trim1)
        }),
    ))
}

fn surface_curve_arguments(entity: &StepEntity) -> Option<&[StepValue]> {
    match entity {
        StepEntity::Simple { name, args } if name == "SURFACE_CURVE" || name == "SEAM_CURVE" => {
            Some(args)
        }
        StepEntity::Complex(parts) => parts
            .iter()
            .find(|(name, _)| name == "SURFACE_CURVE" || name == "SEAM_CURVE")
            .map(|(_, args)| args.as_slice()),
        _ => None,
    }
}

fn curve3d_reference(id: u32, entities: &HashMap<u32, StepEntity>) -> u32 {
    entities
        .get(&id)
        .and_then(surface_curve_arguments)
        .and_then(|args| match args.get(1) {
            Some(StepValue::Ref(curve)) => Some(*curve),
            _ => None,
        })
        .unwrap_or(id)
}

fn parse_curve(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<GeomCurve, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;

    let curve3d = curve3d_reference(id, entities);
    if curve3d != id {
        return parse_curve(curve3d, entities);
    }

    if let StepEntity::Complex(parts) = ent {
        let bspline_part = parts.iter().find(|(name, _)| name == "B_SPLINE_CURVE");
        let knots_part = parts
            .iter()
            .find(|(name, _)| name == "B_SPLINE_CURVE_WITH_KNOTS");
        let rational_part = parts
            .iter()
            .find(|(name, _)| name == "RATIONAL_B_SPLINE_CURVE");

        if let (Some((_, b_args)), Some((_, k_args))) = (bspline_part, knots_part) {
            let degree = match b_args.first() {
                Some(StepValue::Integer(i)) => *i as usize,
                _ => 1,
            };
            let poles_list = match b_args.get(1) {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid B_SPLINE_CURVE poles".to_string()),
            };

            let mut poles = Vec::new();
            for p_val in poles_list {
                if let StepValue::Ref(p_ref) = p_val {
                    poles.push(parse_point(*p_ref, entities)?);
                }
            }

            let mults_list = match k_args.first() {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid B_SPLINE_CURVE multiplicities".to_string()),
            };
            let mut mults = Vec::new();
            for m_val in mults_list {
                match m_val {
                    StepValue::Integer(i) => mults.push(*i as usize),
                    _ => return Err("Invalid multiplicity".to_string()),
                }
            }

            let knots_list = match k_args.get(1) {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid B_SPLINE_CURVE knots".to_string()),
            };
            let mut knots = Vec::new();
            for k_val in knots_list {
                match k_val {
                    StepValue::Real(f) => knots.push(*f),
                    StepValue::Integer(i) => knots.push(*i as f64),
                    _ => return Err("Invalid knot".to_string()),
                }
            }

            let weights = if let Some((_, r_args)) = rational_part {
                let w_list = match r_args.first() {
                    Some(StepValue::List(l)) => l,
                    _ => return Err("Invalid RATIONAL_B_SPLINE_CURVE weights".to_string()),
                };
                let mut w = Vec::new();
                for w_val in w_list {
                    match w_val {
                        StepValue::Real(f) => w.push(*f),
                        StepValue::Integer(i) => w.push(*i as f64),
                        _ => return Err("Invalid weight".to_string()),
                    }
                }
                Some(w)
            } else {
                None
            };

            let curve = BSplineCurve::new(degree, poles, weights, knots, mults);
            return Ok(GeomCurve::BSpline(curve));
        }
    }

    match ent {
        StepEntity::Simple { name, args } => match name.as_str() {
            "LINE" => {
                if args.len() >= 3 {
                    let loc_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid LINE location".to_string()),
                    };
                    let vec_ref = match args[2] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid LINE vector".to_string()),
                    };
                    let loc = parse_point(loc_ref, entities)?;
                    let (dir, _) = parse_vector(vec_ref, entities)?;
                    use openrcad_foundation::Ax1;
                    use openrcad_geom::Line;
                    return Ok(GeomCurve::Line(Line::new(Ax1::new(loc, dir))));
                }
                Err("Invalid LINE arguments".to_string())
            }
            "CIRCLE" => {
                if args.len() >= 3 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid CIRCLE axis".to_string()),
                    };
                    let radius = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::Circle;
                    return Ok(GeomCurve::Circle(Circle::new(axis, radius)));
                }
                Err("Invalid CIRCLE arguments".to_string())
            }
            "ELLIPSE" => {
                if args.len() >= 4 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid ELLIPSE axis".to_string()),
                    };
                    let major_r = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let minor_r = match args[3] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::Ellipse;
                    return Ok(GeomCurve::Ellipse(Ellipse::new(axis, major_r, minor_r)));
                }
                Err("Invalid ELLIPSE arguments".to_string())
            }
            "PARABOLA" => {
                if args.len() >= 3 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid PARABOLA axis".to_string()),
                    };
                    let focal = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::Parabola;
                    return Ok(GeomCurve::Parabola(Parabola::new(axis, focal)));
                }
                Err("Invalid PARABOLA arguments".to_string())
            }
            "HYPERBOLA" => {
                if args.len() >= 4 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid HYPERBOLA axis".to_string()),
                    };
                    let major_r = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let minor_r = match args[3] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::Hyperbola;
                    return Ok(GeomCurve::Hyperbola(Hyperbola::new(axis, major_r, minor_r)));
                }
                Err("Invalid HYPERBOLA arguments".to_string())
            }
            "B_SPLINE_CURVE_WITH_KNOTS" => {
                if args.len() >= 10 {
                    let degree = match args[1] {
                        StepValue::Integer(i) => i as usize,
                        _ => 1,
                    };
                    let poles_list = match &args[2] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid B_SPLINE_CURVE poles".to_string()),
                    };
                    let mut poles = Vec::new();
                    for p_val in poles_list {
                        if let StepValue::Ref(p_ref) = p_val {
                            poles.push(parse_point(*p_ref, entities)?);
                        }
                    }
                    let mults_list = match &args[6] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid B_SPLINE_CURVE multiplicities".to_string()),
                    };
                    let mut mults = Vec::new();
                    for m_val in mults_list {
                        match m_val {
                            StepValue::Integer(i) => mults.push(*i as usize),
                            _ => return Err("Invalid multiplicity".to_string()),
                        }
                    }
                    let knots_list = match &args[7] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid B_SPLINE_CURVE knots".to_string()),
                    };
                    let mut knots = Vec::new();
                    for k_val in knots_list {
                        match k_val {
                            StepValue::Real(f) => knots.push(*f),
                            StepValue::Integer(i) => knots.push(*i as f64),
                            _ => return Err("Invalid knot".to_string()),
                        }
                    }
                    let curve = BSplineCurve::new(degree, poles, None, knots, mults);
                    return Ok(GeomCurve::BSpline(curve));
                }
                Err("Invalid B_SPLINE_CURVE_WITH_KNOTS arguments".to_string())
            }
            _ => Err(format!("Unsupported curve type: {}", name)),
        },
        _ => Err(format!(
            "Expected simple entity or complex entity for curve at #{}",
            id
        )),
    }
}

fn parse_surface(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<GeomSurface, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;

    if let StepEntity::Complex(parts) = ent {
        let bspline_part = parts.iter().find(|(name, _)| name == "B_SPLINE_SURFACE");
        let knots_part = parts
            .iter()
            .find(|(name, _)| name == "B_SPLINE_SURFACE_WITH_KNOTS");
        let rational_part = parts
            .iter()
            .find(|(name, _)| name == "RATIONAL_B_SPLINE_SURFACE");

        if let (Some((_, b_args)), Some((_, k_args))) = (bspline_part, knots_part) {
            let u_degree = match b_args.first() {
                Some(StepValue::Integer(i)) => *i as usize,
                _ => 1,
            };
            let v_degree = match b_args.get(1) {
                Some(StepValue::Integer(i)) => *i as usize,
                _ => 1,
            };

            let poles_list = match b_args.get(2) {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid B_SPLINE_SURFACE poles".to_string()),
            };
            let mut poles = Vec::new();
            for row_val in poles_list {
                if let StepValue::List(row_list) = row_val {
                    let mut row = Vec::new();
                    for p_val in row_list {
                        if let StepValue::Ref(p_ref) = p_val {
                            row.push(parse_point(*p_ref, entities)?);
                        }
                    }
                    poles.push(row);
                }
            }

            let u_mults_list = match k_args.first() {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid U multiplicities".to_string()),
            };
            let mut u_mults = Vec::new();
            for m_val in u_mults_list {
                match m_val {
                    StepValue::Integer(i) => u_mults.push(*i as usize),
                    _ => return Err("Invalid U multiplicity".to_string()),
                }
            }

            let v_mults_list = match k_args.get(1) {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid V multiplicities".to_string()),
            };
            let mut v_mults = Vec::new();
            for m_val in v_mults_list {
                match m_val {
                    StepValue::Integer(i) => v_mults.push(*i as usize),
                    _ => return Err("Invalid V multiplicity".to_string()),
                }
            }

            let u_knots_list = match k_args.get(2) {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid U knots".to_string()),
            };
            let mut u_knots = Vec::new();
            for k_val in u_knots_list {
                match k_val {
                    StepValue::Real(f) => u_knots.push(*f),
                    StepValue::Integer(i) => u_knots.push(*i as f64),
                    _ => return Err("Invalid U knot".to_string()),
                }
            }

            let v_knots_list = match k_args.get(3) {
                Some(StepValue::List(l)) => l,
                _ => return Err("Invalid V knots".to_string()),
            };
            let mut v_knots = Vec::new();
            for k_val in v_knots_list {
                match k_val {
                    StepValue::Real(f) => v_knots.push(*f),
                    StepValue::Integer(i) => v_knots.push(*i as f64),
                    _ => return Err("Invalid V knot".to_string()),
                }
            }

            let weights = if let Some((_, r_args)) = rational_part {
                let w_list = match r_args.first() {
                    Some(StepValue::List(l)) => l,
                    _ => return Err("Invalid RATIONAL_B_SPLINE_SURFACE weights".to_string()),
                };
                let mut w = Vec::new();
                for row_val in w_list {
                    if let StepValue::List(row_list) = row_val {
                        let mut row = Vec::new();
                        for w_val in row_list {
                            match w_val {
                                StepValue::Real(f) => row.push(*f),
                                StepValue::Integer(i) => row.push(*i as f64),
                                _ => return Err("Invalid weight".to_string()),
                            }
                        }
                        w.push(row);
                    }
                }
                Some(w)
            } else {
                None
            };

            let surface = BSplineSurface::new(
                u_degree, v_degree, poles, weights, u_knots, u_mults, v_knots, v_mults,
            );
            return Ok(GeomSurface::BSpline(surface));
        }
    }

    match ent {
        StepEntity::Simple { name, args } => match name.as_str() {
            "PLANE" => {
                if args.len() >= 2 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid PLANE axis".to_string()),
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::Plane;
                    return Ok(GeomSurface::Plane(Plane::new(axis)));
                }
                Err("Invalid PLANE arguments".to_string())
            }
            "CYLINDRICAL_SURFACE" => {
                if args.len() >= 3 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid CYLINDRICAL_SURFACE axis".to_string()),
                    };
                    let radius = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::CylindricalSurface;
                    return Ok(GeomSurface::Cylinder(CylindricalSurface::new(axis, radius)));
                }
                Err("Invalid CYLINDRICAL_SURFACE arguments".to_string())
            }
            "CONICAL_SURFACE" => {
                if args.len() >= 4 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid CONICAL_SURFACE axis".to_string()),
                    };
                    let radius = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let semi_angle = match args[3] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::ConicalSurface;
                    return Ok(GeomSurface::Cone(ConicalSurface::new(
                        axis, radius, semi_angle,
                    )));
                }
                Err("Invalid CONICAL_SURFACE arguments".to_string())
            }
            "SPHERICAL_SURFACE" => {
                if args.len() >= 3 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid SPHERICAL_SURFACE axis".to_string()),
                    };
                    let radius = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::SphericalSurface;
                    return Ok(GeomSurface::Sphere(SphericalSurface::new(axis, radius)));
                }
                Err("Invalid SPHERICAL_SURFACE arguments".to_string())
            }
            "TOROIDAL_SURFACE" => {
                if args.len() >= 4 {
                    let axis_ref = match args[1] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid TOROIDAL_SURFACE axis".to_string()),
                    };
                    let major = match args[2] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let minor = match args[3] {
                        StepValue::Real(f) => f,
                        StepValue::Integer(i) => i as f64,
                        _ => 0.0,
                    };
                    let axis = parse_axis2(axis_ref, entities)?;
                    use openrcad_geom::ToroidalSurface;
                    return Ok(GeomSurface::Torus(ToroidalSurface::new(axis, major, minor)));
                }
                Err("Invalid TOROIDAL_SURFACE arguments".to_string())
            }
            "B_SPLINE_SURFACE_WITH_KNOTS" => {
                if args.len() >= 13 {
                    let u_degree = match args[1] {
                        StepValue::Integer(i) => i as usize,
                        _ => 1,
                    };
                    let v_degree = match args[2] {
                        StepValue::Integer(i) => i as usize,
                        _ => 1,
                    };

                    let poles_list = match &args[3] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid B_SPLINE_SURFACE poles".to_string()),
                    };
                    let mut poles = Vec::new();
                    for row_val in poles_list {
                        if let StepValue::List(row_list) = row_val {
                            let mut row = Vec::new();
                            for p_val in row_list {
                                if let StepValue::Ref(p_ref) = p_val {
                                    row.push(parse_point(*p_ref, entities)?);
                                }
                            }
                            poles.push(row);
                        }
                    }

                    let u_mults_list = match &args[8] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid U multiplicities".to_string()),
                    };
                    let mut u_mults = Vec::new();
                    for m_val in u_mults_list {
                        match m_val {
                            StepValue::Integer(i) => u_mults.push(*i as usize),
                            _ => return Err("Invalid U multiplicity".to_string()),
                        }
                    }

                    let v_mults_list = match &args[9] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid V multiplicities".to_string()),
                    };
                    let mut v_mults = Vec::new();
                    for m_val in v_mults_list {
                        match m_val {
                            StepValue::Integer(i) => v_mults.push(*i as usize),
                            _ => return Err("Invalid V multiplicity".to_string()),
                        }
                    }

                    let u_knots_list = match &args[10] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid U knots".to_string()),
                    };
                    let mut u_knots = Vec::new();
                    for k_val in u_knots_list {
                        match k_val {
                            StepValue::Real(f) => u_knots.push(*f),
                            StepValue::Integer(i) => u_knots.push(*i as f64),
                            _ => return Err("Invalid U knot".to_string()),
                        }
                    }

                    let v_knots_list = match &args[11] {
                        StepValue::List(l) => l,
                        _ => return Err("Invalid V knots".to_string()),
                    };
                    let mut v_knots = Vec::new();
                    for k_val in v_knots_list {
                        match k_val {
                            StepValue::Real(f) => v_knots.push(*f),
                            StepValue::Integer(i) => v_knots.push(*i as f64),
                            _ => return Err("Invalid V knot".to_string()),
                        }
                    }

                    let surface = BSplineSurface::new(
                        u_degree, v_degree, poles, None, u_knots, u_mults, v_knots, v_mults,
                    );
                    return Ok(GeomSurface::BSpline(surface));
                }
                Err("Invalid B_SPLINE_SURFACE_WITH_KNOTS arguments".to_string())
            }
            _ => Err(format!("Unsupported surface type: {}", name)),
        },
        _ => Err(format!(
            "Expected simple entity or complex entity for surface at #{}",
            id
        )),
    }
}

fn project_on_curve(curve: &GeomCurve, p: Pnt) -> f64 {
    match curve {
        GeomCurve::Line(l) => {
            let loc = l.location();
            let dir = l.direction();
            let v =
                openrcad_foundation::Vec::new(p.x() - loc.x(), p.y() - loc.y(), p.z() - loc.z());
            v.dot(&openrcad_foundation::Vec::from_dir(dir))
        }
        GeomCurve::Circle(c) => {
            let loc = c.center();
            let x = c.position().x_direction();
            let y = c.position().y_direction();
            let v =
                openrcad_foundation::Vec::new(p.x() - loc.x(), p.y() - loc.y(), p.z() - loc.z());
            let dx = v.dot(&openrcad_foundation::Vec::from_dir(x));
            let dy = v.dot(&openrcad_foundation::Vec::from_dir(y));
            let mut u = dy.atan2(dx);
            if u < 0.0 {
                u += 2.0 * std::f64::consts::PI;
            }
            u
        }
        GeomCurve::Ellipse(e) => {
            let loc = e.center();
            let x = e.position().x_direction();
            let y = e.position().y_direction();
            let v =
                openrcad_foundation::Vec::new(p.x() - loc.x(), p.y() - loc.y(), p.z() - loc.z());
            let dx = v.dot(&openrcad_foundation::Vec::from_dir(x));
            let dy = v.dot(&openrcad_foundation::Vec::from_dir(y));
            let mut u = (dy / e.minor_radius()).atan2(dx / e.major_radius());
            if u < 0.0 {
                u += 2.0 * std::f64::consts::PI;
            }
            u
        }
        GeomCurve::Helix(h) => {
            // Angle from the radial component, disambiguated across turns by
            // the axial height (u ≈ height·2π/lead).
            let pos = h.position();
            let loc = pos.location();
            let v =
                openrcad_foundation::Vec::new(p.x() - loc.x(), p.y() - loc.y(), p.z() - loc.z());
            let tau = 2.0 * std::f64::consts::PI;
            let axial = v.dot(&openrcad_foundation::Vec::from_dir(pos.direction()));
            let u_height = if h.lead().abs() > 1e-12 {
                axial * tau / h.lead()
            } else {
                0.0
            };
            let dx = v.dot(&openrcad_foundation::Vec::from_dir(pos.x_direction()));
            let dy = v.dot(&openrcad_foundation::Vec::from_dir(pos.y_direction()));
            let ang = dy.atan2(dx);
            let k = ((u_height - ang) / tau).round();
            ang + tau * k
        }
        GeomCurve::Parabola(pa) => {
            let loc = pa.position().location();
            let y = pa.position().y_direction();
            let v =
                openrcad_foundation::Vec::new(p.x() - loc.x(), p.y() - loc.y(), p.z() - loc.z());

            v.dot(&openrcad_foundation::Vec::from_dir(y))
        }
        GeomCurve::Hyperbola(h) => {
            let loc = h.position().location();
            let y = h.position().y_direction();
            let v =
                openrcad_foundation::Vec::new(p.x() - loc.x(), p.y() - loc.y(), p.z() - loc.z());
            let dy = v.dot(&openrcad_foundation::Vec::from_dir(y));
            let val = dy / h.minor_radius();
            val.asinh()
        }
        GeomCurve::BSpline(b) => {
            let (first, last) = b.bounds();
            let mut best_u = first;
            let mut best_dist_sq = f64::INFINITY;
            let n = 100;
            for i in 0..=n {
                let u = first + (last - first) * (i as f64) / (n as f64);
                let pt = b.point(u);
                let dist_sq = pt.distance_squared(&p);
                if dist_sq < best_dist_sq {
                    best_dist_sq = dist_sq;
                    best_u = u;
                }
            }
            let mut u = best_u;
            for _ in 0..5 {
                let (pt, tangent): (Pnt, openrcad_foundation::Vec) = b.d1(u);
                let diff =
                    openrcad_foundation::Vec::new(pt.x() - p.x(), pt.y() - p.y(), pt.z() - p.z());
                let f_val = diff.dot(&tangent);
                let f_prime = tangent.magnitude_squared();
                if f_prime.abs() > 1e-12 {
                    let next_u = u - f_val / f_prime;
                    if next_u >= first && next_u <= last {
                        u = next_u;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
            u
        }
    }
}

fn project_on_curve2d(curve: &GeomCurve2d, point: Pnt2d) -> Option<f64> {
    match curve {
        GeomCurve2d::Line(line) => {
            let delta = point - line.location();
            let direction = line.direction();
            Some(delta.x() * direction.x() + delta.y() * direction.y())
        }
        GeomCurve2d::Circle(circle) => {
            let delta = point - circle.center();
            let x = circle.position().x_direction();
            let y = circle.position().y_direction();
            let local_x = delta.x() * x.x() + delta.y() * x.y();
            let local_y = delta.x() * y.x() + delta.y() * y.y();
            Some(local_y.atan2(local_x).rem_euclid(core::f64::consts::TAU))
        }
        GeomCurve2d::Ellipse(ellipse) => {
            let delta = point - ellipse.center();
            let x = ellipse.position().x_direction();
            let y = ellipse.position().y_direction();
            let local_x = (delta.x() * x.x() + delta.y() * x.y()) / ellipse.major_radius();
            let local_y = (delta.x() * y.x() + delta.y() * y.y()) / ellipse.minor_radius();
            Some(local_y.atan2(local_x).rem_euclid(core::f64::consts::TAU))
        }
        _ => {
            let (first, last) = curve.bounds();
            if !first.is_finite() || !last.is_finite() || first == last {
                return None;
            }
            let samples = 96;
            let mut best_parameter = first;
            let mut best_distance = f64::INFINITY;
            for index in 0..=samples {
                let parameter = first + (last - first) * index as f64 / samples as f64;
                let distance = curve.point(parameter).distance(&point);
                if distance < best_distance {
                    best_distance = distance;
                    best_parameter = parameter;
                }
            }
            let step = (last - first).abs() / samples as f64;
            let mut left = (best_parameter - step).max(first.min(last));
            let mut right = (best_parameter + step).min(first.max(last));
            for _ in 0..24 {
                let p1 = left + (right - left) / 3.0;
                let p2 = right - (right - left) / 3.0;
                if curve.point(p1).distance(&point) <= curve.point(p2).distance(&point) {
                    right = p2;
                } else {
                    left = p1;
                }
            }
            Some(0.5 * (left + right))
        }
    }
}

fn periodicity_for_surface(surface: &GeomSurface) -> SurfacePeriodicity {
    SurfacePeriodicity {
        u_period: surface.is_uclosed().then_some(core::f64::consts::TAU),
        v_period: surface.is_vclosed().then_some(core::f64::consts::TAU),
    }
}

fn align_uv_to_pcurve(
    mut uv: Pnt2d,
    curve: &GeomCurve2d,
    periodicity: SurfacePeriodicity,
) -> Pnt2d {
    let (first, last) = curve.bounds();
    let reference_parameter = if first.is_finite() && last.is_finite() {
        0.5 * (first + last)
    } else {
        0.0
    };
    let reference = curve.point(reference_parameter);
    if let Some(period) = periodicity.u_period {
        uv = Pnt2d::new(
            uv.x() + ((reference.x() - uv.x()) / period).round() * period,
            uv.y(),
        );
    }
    if let Some(period) = periodicity.v_period {
        uv = Pnt2d::new(
            uv.x(),
            uv.y() + ((reference.y() - uv.y()) / period).round() * period,
        );
    }
    uv
}

fn representation_curve2d_reference(
    representation: u32,
    entities: &HashMap<u32, StepEntity>,
) -> Option<u32> {
    match entities.get(&representation) {
        Some(StepEntity::Simple { name, args })
            if name == "DEFINITIONAL_REPRESENTATION" || name == "REPRESENTATION" =>
        {
            match args.get(1) {
                Some(StepValue::List(items)) => items.iter().find_map(|item| match item {
                    StepValue::Ref(id) => Some(*id),
                    _ => None,
                }),
                _ => None,
            }
        }
        Some(_) => Some(representation),
        None => None,
    }
}

fn parse_associated_pcurve(
    surface_curve: u32,
    surface_ref: u32,
    surface: &GeomSurface,
    edge_start: Pnt,
    edge_end: Pnt,
    entities: &HashMap<u32, StepEntity>,
) -> Option<PcurveData> {
    let arguments = entities
        .get(&surface_curve)
        .and_then(surface_curve_arguments)?;
    let associated = match arguments.get(2) {
        Some(StepValue::List(values)) => values,
        _ => return None,
    };
    let periodicity = periodicity_for_surface(surface);

    for associated_geometry in associated {
        let StepValue::Ref(pcurve_ref) = associated_geometry else {
            continue;
        };
        let Some(StepEntity::Simple { name, args }) = entities.get(pcurve_ref) else {
            continue;
        };
        if name != "PCURVE" {
            continue;
        }
        let basis_surface = match args.get(1) {
            Some(StepValue::Ref(id)) => *id,
            _ => continue,
        };
        if basis_surface != surface_ref {
            continue;
        }
        let representation = match args.get(2) {
            Some(StepValue::Ref(id)) => *id,
            _ => continue,
        };
        let Some(curve_ref) = representation_curve2d_reference(representation, entities) else {
            continue;
        };
        let Ok((curve, exact_range)) = parse_curve2d_representation(curve_ref, entities) else {
            continue;
        };

        let (mut first, mut last) = if let Some(range) = exact_range {
            range
        } else {
            let start_uv = openrcad_mesh::triangulate::project_point(surface, edge_start, None);
            let end_uv =
                openrcad_mesh::triangulate::project_point(surface, edge_end, Some(start_uv));
            let start_uv =
                align_uv_to_pcurve(Pnt2d::new(start_uv.0, start_uv.1), &curve, periodicity);
            let end_uv = align_uv_to_pcurve(Pnt2d::new(end_uv.0, end_uv.1), &curve, periodicity);
            let Some(first) = project_on_curve2d(&curve, start_uv) else {
                continue;
            };
            let Some(last) = project_on_curve2d(&curve, end_uv) else {
                continue;
            };
            (first, last)
        };

        // STEP trim senses describe the pcurve's basis direction. Align the
        // exact interval with this topological edge's natural start/end.
        let first_uv = curve.point(first);
        let last_uv = curve.point(last);
        let first_point = surface.point(first_uv.x(), first_uv.y());
        let last_point = surface.point(last_uv.x(), last_uv.y());
        if first_point.distance(&edge_start) > last_point.distance(&edge_start) {
            core::mem::swap(&mut first, &mut last);
        }

        if (last - first).abs() <= openrcad_foundation::tolerance::CONFUSION && curve.is_periodic()
        {
            (first, last) = curve.bounds();
        }
        let pcurve = PcurveData::new(curve, first, last).with_periodicity(periodicity);
        if pcurve.is_valid() {
            return Some(pcurve);
        }
    }
    None
}

fn reconstruct_brep(entities: HashMap<u32, StepEntity>, shell_id: u32) -> Result<Solid, String> {
    let mut brep = BRep::new();

    let mut vertex_map = HashMap::new();
    let mut edge_map = HashMap::new();
    let mut loop_map = HashMap::new();
    let mut face_map = HashMap::new();

    let shell_ent = entities
        .get(&shell_id)
        .ok_or_else(|| format!("Shell #{} not found", shell_id))?;
    let face_refs = match shell_ent {
        StepEntity::Simple { name, args } if name == "CLOSED_SHELL" => {
            if args.len() >= 2 {
                match &args[1] {
                    StepValue::List(l) => l
                        .iter()
                        .map(|v| match v {
                            StepValue::Ref(r) => Ok(*r),
                            _ => Err("Invalid face reference in shell".to_string()),
                        })
                        .collect::<Result<Vec<u32>, String>>()?,
                    _ => return Err("Invalid CLOSED_SHELL face list".to_string()),
                }
            } else {
                return Err("Invalid CLOSED_SHELL arguments".to_string());
            }
        }
        _ => {
            return Err(format!(
                "Expected CLOSED_SHELL at #{}, found {:?}",
                shell_id, shell_ent
            ))
        }
    };

    let mut shell_faces = Vec::new();

    for &face_ref in &face_refs {
        if let Some(&f_id) = face_map.get(&face_ref) {
            shell_faces.push(f_id);
            continue;
        }

        let face_ent = entities
            .get(&face_ref)
            .ok_or_else(|| format!("Face #{} not found", face_ref))?;
        let (bounds_list, surface_ref, same_sense_val) = match face_ent {
            StepEntity::Simple { name, args } if name == "ADVANCED_FACE" => {
                if args.len() >= 4 {
                    let bounds = match &args[1] {
                        StepValue::List(l) => l
                            .iter()
                            .map(|v| match v {
                                StepValue::Ref(r) => Ok(*r),
                                _ => Err("Invalid bound reference".to_string()),
                            })
                            .collect::<Result<Vec<u32>, String>>()?,
                        _ => return Err("Invalid ADVANCED_FACE bounds".to_string()),
                    };
                    let surface_ref = match args[2] {
                        StepValue::Ref(r) => r,
                        _ => return Err("Invalid ADVANCED_FACE surface".to_string()),
                    };
                    let same_sense = match &args[3] {
                        StepValue::Enum(s) => s == "T",
                        _ => true,
                    };
                    (bounds, surface_ref, same_sense)
                } else {
                    return Err("Invalid ADVANCED_FACE arguments".to_string());
                }
            }
            _ => {
                return Err(format!(
                    "Expected ADVANCED_FACE at #{}, found {:?}",
                    face_ref, face_ent
                ))
            }
        };

        let surface = parse_surface(surface_ref, &entities)?;

        let mut outer_wire = None;
        let mut inner_wires = Vec::new();

        for &bound_ref in &bounds_list {
            let bound_ent = entities
                .get(&bound_ref)
                .ok_or_else(|| format!("Bound #{} not found", bound_ref))?;
            let (loop_ref, is_outer) = match bound_ent {
                StepEntity::Simple { name, args }
                    if name == "FACE_OUTER_BOUND" || name == "FACE_BOUND" =>
                {
                    if args.len() >= 3 {
                        let l_ref = match args[1] {
                            StepValue::Ref(r) => r,
                            _ => return Err("Invalid bound loop reference".to_string()),
                        };
                        (l_ref, name == "FACE_OUTER_BOUND")
                    } else {
                        return Err("Invalid bound arguments".to_string());
                    }
                }
                _ => {
                    return Err(format!(
                        "Expected FACE_OUTER_BOUND or FACE_BOUND at #{}, found {:?}",
                        bound_ref, bound_ent
                    ))
                }
            };

            let l_id = if let Some(&l_id) = loop_map.get(&loop_ref) {
                l_id
            } else {
                let loop_ent = entities
                    .get(&loop_ref)
                    .ok_or_else(|| format!("Loop #{} not found", loop_ref))?;
                let oe_refs = match loop_ent {
                    StepEntity::Simple { name, args } if name == "EDGE_LOOP" => {
                        if args.len() >= 2 {
                            match &args[1] {
                                StepValue::List(l) => l
                                    .iter()
                                    .map(|v| match v {
                                        StepValue::Ref(r) => Ok(*r),
                                        _ => Err("Invalid oriented edge reference".to_string()),
                                    })
                                    .collect::<Result<Vec<u32>, String>>()?,
                                _ => return Err("Invalid EDGE_LOOP list".to_string()),
                            }
                        } else {
                            return Err("Invalid EDGE_LOOP arguments".to_string());
                        }
                    }
                    _ => {
                        return Err(format!(
                            "Expected EDGE_LOOP at #{}, found {:?}",
                            loop_ref, loop_ent
                        ))
                    }
                };

                let mut oriented_edges = Vec::new();
                for &oe_ref in &oe_refs {
                    let oe_ent = entities
                        .get(&oe_ref)
                        .ok_or_else(|| format!("Oriented edge #{} not found", oe_ref))?;
                    let (edge_ref, orientation_forward) = match oe_ent {
                        StepEntity::Simple { name, args } if name == "ORIENTED_EDGE" => {
                            if args.len() >= 5 {
                                let e_ref = match args[3] {
                                    StepValue::Ref(r) => r,
                                    _ => {
                                        return Err(
                                            "Invalid ORIENTED_EDGE edge reference".to_string()
                                        )
                                    }
                                };
                                let same_sense = match &args[4] {
                                    StepValue::Enum(s) => s == "T",
                                    _ => true,
                                };
                                (e_ref, same_sense)
                            } else {
                                return Err("Invalid ORIENTED_EDGE arguments".to_string());
                            }
                        }
                        _ => {
                            return Err(format!(
                                "Expected ORIENTED_EDGE at #{}, found {:?}",
                                oe_ref, oe_ent
                            ))
                        }
                    };

                    let e_id = if let Some(&e_id) = edge_map.get(&edge_ref) {
                        e_id
                    } else {
                        let edge_ent = entities
                            .get(&edge_ref)
                            .ok_or_else(|| format!("Edge #{} not found", edge_ref))?;
                        let (v1_ref, v2_ref, curve_ref, curve_same_sense) = match edge_ent {
                            StepEntity::Simple { name, args } if name == "EDGE_CURVE" => {
                                if args.len() >= 5 {
                                    let v1 = match args[1] {
                                        StepValue::Ref(r) => r,
                                        _ => return Err("Invalid start vertex".to_string()),
                                    };
                                    let v2 = match args[2] {
                                        StepValue::Ref(r) => r,
                                        _ => return Err("Invalid end vertex".to_string()),
                                    };
                                    let c = match args[3] {
                                        StepValue::Ref(r) => r,
                                        _ => return Err("Invalid edge curve".to_string()),
                                    };
                                    let same_sense = match &args[4] {
                                        StepValue::Enum(s) => s == "T",
                                        _ => true,
                                    };
                                    (v1, v2, c, same_sense)
                                } else {
                                    return Err("Invalid EDGE_CURVE arguments".to_string());
                                }
                            }
                            _ => {
                                return Err(format!(
                                    "Expected EDGE_CURVE at #{}, found {:?}",
                                    edge_ref, edge_ent
                                ))
                            }
                        };

                        let start_v_id = if let Some(&v_id) = vertex_map.get(&v1_ref) {
                            v_id
                        } else {
                            let v1_ent = entities
                                .get(&v1_ref)
                                .ok_or_else(|| format!("Vertex #{} not found", v1_ref))?;
                            let pt_ref = match v1_ent {
                                StepEntity::Simple { name, args } if name == "VERTEX_POINT" => {
                                    if args.len() >= 2 {
                                        match args[1] {
                                            StepValue::Ref(r) => r,
                                            _ => {
                                                return Err("Invalid VERTEX_POINT point reference"
                                                    .to_string())
                                            }
                                        }
                                    } else {
                                        return Err("Invalid VERTEX_POINT arguments".to_string());
                                    }
                                }
                                _ => {
                                    return Err(format!(
                                        "Expected VERTEX_POINT at #{}, found {:?}",
                                        v1_ref, v1_ent
                                    ))
                                }
                            };
                            let pt = parse_point(pt_ref, &entities)?;
                            let v_id = brep.vertices.insert(VertexData {
                                point: pt,
                                tolerance: openrcad_foundation::tolerance::CONFUSION,
                            });
                            vertex_map.insert(v1_ref, v_id);
                            v_id
                        };

                        let end_v_id = if let Some(&v_id) = vertex_map.get(&v2_ref) {
                            v_id
                        } else {
                            let v2_ent = entities
                                .get(&v2_ref)
                                .ok_or_else(|| format!("Vertex #{} not found", v2_ref))?;
                            let pt_ref = match v2_ent {
                                StepEntity::Simple { name, args } if name == "VERTEX_POINT" => {
                                    if args.len() >= 2 {
                                        match args[1] {
                                            StepValue::Ref(r) => r,
                                            _ => {
                                                return Err("Invalid VERTEX_POINT point reference"
                                                    .to_string())
                                            }
                                        }
                                    } else {
                                        return Err("Invalid VERTEX_POINT arguments".to_string());
                                    }
                                }
                                _ => {
                                    return Err(format!(
                                        "Expected VERTEX_POINT at #{}, found {:?}",
                                        v2_ref, v2_ent
                                    ))
                                }
                            };
                            let pt = parse_point(pt_ref, &entities)?;
                            let v_id = brep.vertices.insert(VertexData {
                                point: pt,
                                tolerance: openrcad_foundation::tolerance::CONFUSION,
                            });
                            vertex_map.insert(v2_ref, v_id);
                            v_id
                        };

                        let curve = parse_curve(curve_ref, &entities)?;
                        let first = project_on_curve(&curve, brep.vertices[start_v_id].point);
                        let last = project_on_curve(&curve, brep.vertices[end_v_id].point);

                        // The edge is stored in its natural sense (start -> end with the
                        // projected first/last params). The EDGE_CURVE `same_sense` flag is
                        // recovered on export from whether `first <= last`; loop-traversal
                        // orientation lives per-use in each ORIENTED_EDGE.
                        let _ = curve_same_sense;
                        let e_id = brep.edges.insert(EdgeData {
                            curve: Some(curve),
                            first,
                            last,
                            start: start_v_id,
                            end: end_v_id,
                            tolerance: openrcad_foundation::tolerance::CONFUSION,
                        });
                        edge_map.insert(edge_ref, e_id);
                        e_id
                    };

                    let surface_curve_ref = match entities.get(&edge_ref) {
                        Some(StepEntity::Simple { name, args }) if name == "EDGE_CURVE" => {
                            match args.get(3) {
                                Some(StepValue::Ref(curve)) => *curve,
                                _ => return Err("Invalid edge curve".to_string()),
                            }
                        }
                        _ => return Err(format!("Expected EDGE_CURVE at #{edge_ref}")),
                    };
                    let edge_data = &brep.edges[e_id];
                    let pcurve = parse_associated_pcurve(
                        surface_curve_ref,
                        surface_ref,
                        &surface,
                        brep.vertices[edge_data.start].point,
                        brep.vertices[edge_data.end].point,
                        &entities,
                    )
                    .map(|data| brep.pcurves.insert(data));

                    oriented_edges.push(OrientedEdge {
                        id: e_id,
                        orientation: if orientation_forward {
                            Orientation::Forward
                        } else {
                            Orientation::Reversed
                        },
                        pcurve,
                    });
                }

                let l_id = brep.loops.insert(LoopData {
                    edges: oriented_edges,
                });
                loop_map.insert(loop_ref, l_id);
                l_id
            };

            if is_outer {
                outer_wire = Some(l_id);
            } else {
                inner_wires.push(l_id);
            }
        }

        let f_id = brep.faces.insert(FaceData {
            surface: Some(surface),
            outer_wire,
            inner_wires,
            orientation: if same_sense_val {
                Orientation::Forward
            } else {
                Orientation::Reversed
            },
        });
        face_map.insert(face_ref, f_id);
        shell_faces.push(f_id);
    }

    let shell_id_new = brep.shells.insert(ShellData { faces: shell_faces });

    let solid_id_new = brep.solids.insert(SolidData {
        shells: vec![shell_id_new],
    });

    Ok(Solid::from_id(std::sync::Arc::new(brep), solid_id_new))
}

/// Read a STEP file at `path` into a [`Solid`] (AP242 B-Rep).
pub fn read_step(path: &str) -> io::Result<Solid> {
    let content = fs::read_to_string(path)?;
    read_step_str(&content)
}

/// Parse STEP file text into a [`Solid`] (AP242 B-Rep).
///
/// Same semantics as [`read_step`], but takes the file contents directly so
/// callers that embed STEP data (e.g. a document format) avoid the filesystem.
pub fn read_step_str(content: &str) -> io::Result<Solid> {
    let stripped = strip_comments(content);

    let data_start = stripped
        .find("DATA;")
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No DATA section found"))?;
    let data_end = stripped[data_start..]
        .find("ENDSEC;")
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "No ENDSEC after DATA found"))?;

    let data_str = &stripped[data_start + 5..data_start + data_end];
    let tokens = tokenize(data_str).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

    let mut parser = Parser::new(&tokens);
    let mut entities = HashMap::new();

    while parser.peek().is_some() {
        let (id, ent) = parser
            .parse_entity()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        entities.insert(id, ent);
    }

    let solid_ent = entities.iter().find(|(_, ent)| match ent {
        StepEntity::Simple { name, .. } => name == "MANIFOLD_SOLID_BREP",
        _ => false,
    });

    let shell_id = match solid_ent {
        Some((_, StepEntity::Simple { args, .. })) if args.len() >= 2 => match &args[1] {
            StepValue::Ref(r) => *r,
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Invalid MANIFOLD_SOLID_BREP shell reference",
                ))
            }
        },
        _ => {
            let shell_ent = entities.iter().find(|(_, ent)| match ent {
                StepEntity::Simple { name, .. } => name == "CLOSED_SHELL",
                _ => false,
            });
            match shell_ent {
                Some((&id, _)) => id,
                None => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "No MANIFOLD_SOLID_BREP or CLOSED_SHELL found",
                    ))
                }
            }
        }
    };

    let solid = reconstruct_brep(entities, shell_id)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    Ok(solid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entity(name: &str, args: Vec<StepValue>) -> StepEntity {
        StepEntity::Simple {
            name: name.to_string(),
            args,
        }
    }

    fn string() -> StepValue {
        StepValue::String(String::new())
    }

    fn numbers(values: &[f64]) -> StepValue {
        StepValue::List(values.iter().copied().map(StepValue::Real).collect())
    }

    #[test]
    fn surface_curve_import_attaches_face_specific_pcurves() {
        let mut entities = HashMap::new();

        // Shared 3D edge A -> B.
        entities.insert(
            1,
            entity("CARTESIAN_POINT", vec![string(), numbers(&[0.0, 0.0, 0.0])]),
        );
        entities.insert(2, entity("VERTEX_POINT", vec![string(), StepValue::Ref(1)]));
        entities.insert(
            3,
            entity("CARTESIAN_POINT", vec![string(), numbers(&[1.0, 0.0, 0.0])]),
        );
        entities.insert(4, entity("VERTEX_POINT", vec![string(), StepValue::Ref(3)]));
        entities.insert(
            5,
            entity("DIRECTION", vec![string(), numbers(&[1.0, 0.0, 0.0])]),
        );
        entities.insert(
            6,
            entity(
                "VECTOR",
                vec![string(), StepValue::Ref(5), StepValue::Real(1.0)],
            ),
        );
        entities.insert(
            7,
            entity("LINE", vec![string(), StepValue::Ref(1), StepValue::Ref(6)]),
        );

        // XY carrying plane.
        entities.insert(
            20,
            entity("DIRECTION", vec![string(), numbers(&[0.0, 0.0, 1.0])]),
        );
        entities.insert(
            21,
            entity("DIRECTION", vec![string(), numbers(&[1.0, 0.0, 0.0])]),
        );
        entities.insert(
            22,
            entity(
                "AXIS2_PLACEMENT_3D",
                vec![
                    string(),
                    StepValue::Ref(1),
                    StepValue::Ref(20),
                    StepValue::Ref(21),
                ],
            ),
        );
        entities.insert(23, entity("PLANE", vec![string(), StepValue::Ref(22)]));

        // 2D line pcurve and its STEP representation wrapper.
        entities.insert(
            30,
            entity("CARTESIAN_POINT", vec![string(), numbers(&[0.0, 0.0])]),
        );
        entities.insert(
            31,
            entity("DIRECTION", vec![string(), numbers(&[1.0, 0.0])]),
        );
        entities.insert(
            32,
            entity(
                "VECTOR",
                vec![string(), StepValue::Ref(31), StepValue::Real(1.0)],
            ),
        );
        entities.insert(
            33,
            entity(
                "LINE",
                vec![string(), StepValue::Ref(30), StepValue::Ref(32)],
            ),
        );
        entities.insert(
            34,
            entity(
                "DEFINITIONAL_REPRESENTATION",
                vec![
                    string(),
                    StepValue::List(vec![StepValue::Ref(38)]),
                    StepValue::Omitted,
                ],
            ),
        );
        entities.insert(
            38,
            entity(
                "TRIMMED_CURVE",
                vec![
                    string(),
                    StepValue::Ref(33),
                    StepValue::List(vec![StepValue::Typed(
                        "PARAMETER_VALUE".to_string(),
                        Box::new(StepValue::Real(0.0)),
                    )]),
                    StepValue::List(vec![StepValue::Typed(
                        "PARAMETER_VALUE".to_string(),
                        Box::new(StepValue::Real(1.0)),
                    )]),
                    StepValue::Enum("T".to_string()),
                    StepValue::Enum("PARAMETER".to_string()),
                ],
            ),
        );
        entities.insert(
            35,
            entity(
                "PCURVE",
                vec![string(), StepValue::Ref(23), StepValue::Ref(34)],
            ),
        );
        entities.insert(
            36,
            entity(
                "SURFACE_CURVE",
                vec![
                    string(),
                    StepValue::Ref(7),
                    StepValue::List(vec![StepValue::Ref(35)]),
                    StepValue::Enum("PCURVE_S1".to_string()),
                ],
            ),
        );
        entities.insert(
            37,
            entity(
                "EDGE_CURVE",
                vec![
                    string(),
                    StepValue::Ref(2),
                    StepValue::Ref(4),
                    StepValue::Ref(36),
                    StepValue::Enum("T".to_string()),
                ],
            ),
        );

        // Use the edge in both directions to form a minimal closed wire.
        for (id, sense) in [(40, "T"), (41, "F")] {
            entities.insert(
                id,
                entity(
                    "ORIENTED_EDGE",
                    vec![
                        string(),
                        StepValue::Omitted,
                        StepValue::Omitted,
                        StepValue::Ref(37),
                        StepValue::Enum(sense.to_string()),
                    ],
                ),
            );
        }
        entities.insert(
            42,
            entity(
                "EDGE_LOOP",
                vec![
                    string(),
                    StepValue::List(vec![StepValue::Ref(40), StepValue::Ref(41)]),
                ],
            ),
        );
        entities.insert(
            43,
            entity(
                "FACE_OUTER_BOUND",
                vec![
                    string(),
                    StepValue::Ref(42),
                    StepValue::Enum("T".to_string()),
                ],
            ),
        );
        entities.insert(
            44,
            entity(
                "ADVANCED_FACE",
                vec![
                    string(),
                    StepValue::List(vec![StepValue::Ref(43)]),
                    StepValue::Ref(23),
                    StepValue::Enum("T".to_string()),
                ],
            ),
        );
        entities.insert(
            45,
            entity(
                "CLOSED_SHELL",
                vec![string(), StepValue::List(vec![StepValue::Ref(44)])],
            ),
        );

        let solid = reconstruct_brep(entities, 45).expect("reconstruct STEP pcurve fixture");

        assert_eq!(solid.brep().pcurves.len(), 2);
        let face = solid.shell().faces().remove(0);
        let wire = face.outer_wire().unwrap();
        assert!(wire.pcurve(0).is_some());
        assert!(wire.pcurve(1).is_some());
        assert_eq!(
            (wire.pcurve(0).unwrap().first, wire.pcurve(0).unwrap().last),
            (0.0, 1.0)
        );
        assert!(solid.validate().is_ok());
    }
}
