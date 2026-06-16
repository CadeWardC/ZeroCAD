//! A tiny arithmetic expression evaluator for dimension fields: supports
//! `+ - * /`, parentheses, unary minus, decimal numbers, and named variables
//! resolved from a caller-supplied map. Lives in core so the parametric engine
//! can re-evaluate a stored expression against the current variables every time
//! the model is built — that's what makes a dimension follow its variable.
//!
//! Values are evaluated in the **base unit (millimeters)**: numeric literals are
//! mm and a variable contributes its [`crate::Variable::value_in_base`]. This
//! matches how the geometry consumes dimensions, so an expression resolves to the
//! same length the kernel will extrude.

use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Num(f64),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    LParen,
    RParen,
}

fn is_ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_'
}

fn is_ident_char(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

fn tokenize(input: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let chars: Vec<char> = input.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            i += 1;
        } else if c.is_ascii_digit() || c == '.' {
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                i += 1;
            }
            let s: String = chars[start..i].iter().collect();
            let v = s
                .parse::<f64>()
                .map_err(|_| format!("invalid number '{s}'"))?;
            tokens.push(Token::Num(v));
        } else if is_ident_start(c) {
            let start = i;
            while i < chars.len() && is_ident_char(chars[i]) {
                i += 1;
            }
            tokens.push(Token::Ident(chars[start..i].iter().collect()));
        } else {
            tokens.push(match c {
                '+' => Token::Plus,
                '-' => Token::Minus,
                '*' => Token::Star,
                '/' => Token::Slash,
                '(' => Token::LParen,
                ')' => Token::RParen,
                other => return Err(format!("unexpected character '{other}'")),
            });
            i += 1;
        }
    }
    Ok(tokens)
}

/// Evaluate `input` to a number, resolving identifiers via `vars`. Returns
/// `Err` with a short message on malformed input, an unknown name, or a divide
/// by zero. An empty (or whitespace-only) input is an error, so callers keep
/// the last valid value while the user is mid-edit.
pub fn eval(input: &str, vars: &HashMap<String, f64>) -> Result<f64, String> {
    let tokens = tokenize(input)?;
    if tokens.is_empty() {
        return Err("empty expression".into());
    }
    let mut p = Parser {
        tokens: &tokens,
        pos: 0,
        vars,
    };
    let v = p.expr()?;
    if p.pos != p.tokens.len() {
        return Err("unexpected trailing input".into());
    }
    if !v.is_finite() {
        return Err("result is not finite".into());
    }
    Ok(v)
}

struct Parser<'a> {
    tokens: &'a [Token],
    pos: usize,
    vars: &'a HashMap<String, f64>,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<&'a Token> {
        self.tokens.get(self.pos)
    }

    fn bump(&mut self) -> Option<&'a Token> {
        let t = self.tokens.get(self.pos);
        if t.is_some() {
            self.pos += 1;
        }
        t
    }

    // expr := term (('+' | '-') term)*
    fn expr(&mut self) -> Result<f64, String> {
        let mut acc = self.term()?;
        while let Some(op) = self.peek() {
            match op {
                Token::Plus => {
                    self.bump();
                    acc += self.term()?;
                }
                Token::Minus => {
                    self.bump();
                    acc -= self.term()?;
                }
                _ => break,
            }
        }
        Ok(acc)
    }

    // term := factor (('*' | '/') factor)*
    fn term(&mut self) -> Result<f64, String> {
        let mut acc = self.factor()?;
        while let Some(op) = self.peek() {
            match op {
                Token::Star => {
                    self.bump();
                    acc *= self.factor()?;
                }
                Token::Slash => {
                    self.bump();
                    let d = self.factor()?;
                    if d == 0.0 {
                        return Err("division by zero".into());
                    }
                    acc /= d;
                }
                _ => break,
            }
        }
        Ok(acc)
    }

    // factor := ('+' | '-') factor | primary
    fn factor(&mut self) -> Result<f64, String> {
        match self.peek() {
            Some(Token::Minus) => {
                self.bump();
                Ok(-self.factor()?)
            }
            Some(Token::Plus) => {
                self.bump();
                self.factor()
            }
            _ => self.primary(),
        }
    }

    // primary := Num | Ident | '(' expr ')'
    fn primary(&mut self) -> Result<f64, String> {
        match self.bump() {
            Some(Token::Num(v)) => Ok(*v),
            Some(Token::Ident(name)) => self
                .vars
                .get(name)
                .copied()
                .ok_or_else(|| format!("unknown variable '{name}'")),
            Some(Token::LParen) => {
                let v = self.expr()?;
                match self.bump() {
                    Some(Token::RParen) => Ok(v),
                    _ => Err("expected ')'".into()),
                }
            }
            _ => Err("expected a number, variable, or '('".into()),
        }
    }
}

/// True if `input` references at least one variable (contains an identifier).
/// Used to decide whether a dimension is a live expression worth persisting, or
/// just a literal number.
pub fn references_variable(input: &str) -> bool {
    match tokenize(input) {
        Ok(tokens) => tokens.iter().any(|t| matches!(t, Token::Ident(_))),
        Err(_) => input.chars().any(is_ident_start),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vars() -> HashMap<String, f64> {
        [("width".to_string(), 50.0), ("height".to_string(), 20.0)]
            .into_iter()
            .collect()
    }

    fn ev(s: &str) -> f64 {
        eval(s, &vars()).unwrap()
    }

    #[test]
    fn plain_number() {
        assert_eq!(ev("42"), 42.0);
        assert_eq!(ev("3.5"), 3.5);
    }

    #[test]
    fn arithmetic_and_precedence() {
        assert_eq!(ev("2 + 3 * 4"), 14.0);
        assert_eq!(ev("(2 + 3) * 4"), 20.0);
        assert_eq!(ev("10 / 4"), 2.5);
        assert_eq!(ev("10 - 2 - 3"), 5.0);
    }

    #[test]
    fn unary_minus() {
        assert_eq!(ev("-5"), -5.0);
        assert_eq!(ev("3 * -2"), -6.0);
        assert_eq!(ev("-(2 + 3)"), -5.0);
    }

    #[test]
    fn variables_resolve_and_combine() {
        assert_eq!(ev("width"), 50.0);
        assert_eq!(ev("width / 2"), 25.0);
        assert_eq!(ev("width + height"), 70.0);
        assert_eq!(ev("width / 2 + 3"), 28.0);
    }

    #[test]
    fn errors() {
        assert!(eval("", &vars()).is_err());
        assert!(eval("   ", &vars()).is_err());
        assert!(eval("width +", &vars()).is_err());
        assert!(eval("nope", &vars()).is_err());
        assert!(eval("1 / 0", &vars()).is_err());
        assert!(eval("2 3", &vars()).is_err());
        assert!(eval("(1 + 2", &vars()).is_err());
    }

    #[test]
    fn references_variable_detects_identifiers() {
        assert!(references_variable("width"));
        assert!(references_variable("width / 2 + 3"));
        assert!(!references_variable("42"));
        assert!(!references_variable("3.5 * 2"));
    }
}
