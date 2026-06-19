#![forbid(unsafe_code)]
//! STEP AP242 (ISO 10303-21) B-Rep Reader.

use std::collections::HashMap;
use std::fs;
use std::io;

use openrcad_foundation::{Ax2, Ax3, Dir, Pnt};
use openrcad_geom::{BSplineCurve, BSplineSurface, Curve, GeomCurve, GeomSurface};
use openrcad_topo::{
    arena::{BRep, EdgeData, FaceData, LoopData, OrientedEdge, ShellData, SolidData, VertexData},
    orientation::Orientation,
    Solid,
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

fn parse_curve(id: u32, entities: &HashMap<u32, StepEntity>) -> Result<GeomCurve, String> {
    let ent = entities
        .get(&id)
        .ok_or_else(|| format!("Entity #{} not found", id))?;

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

                    oriented_edges.push(OrientedEdge {
                        id: e_id,
                        orientation: if orientation_forward {
                            Orientation::Forward
                        } else {
                            Orientation::Reversed
                        },
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
    let stripped = strip_comments(&content);

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
