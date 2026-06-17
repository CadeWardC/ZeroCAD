#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum Unit {
    #[default]
    Millimeter,
    Inch,
    Meter,
}

impl Unit {
    /// Returns the multiplier to convert this unit to the base unit (Millimeters).
    pub fn to_base_multiplier(self) -> f64 {
        match self {
            Unit::Millimeter => 1.0,
            Unit::Inch => 25.4,
            Unit::Meter => 1000.0,
        }
    }

    /// Converts a value from this unit to the base unit (Millimeters).
    pub fn to_base(self, value: f64) -> f64 {
        value * self.to_base_multiplier()
    }

    /// Converts a value from the base unit (Millimeters) to this unit.
    pub fn from_base(self, value: f64) -> f64 {
        value / self.to_base_multiplier()
    }

    /// Returns the standard suffix for the unit.
    pub fn suffix(self) -> &'static str {
        match self {
            Unit::Millimeter => "mm",
            Unit::Inch => "in",
            Unit::Meter => "m",
        }
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Parameter {
    pub name: String,
    pub expression: String,
    pub value_in_base: f64,
}

impl Parameter {
    pub fn new(name: &str, expression: &str, current_unit: Unit) -> Result<Self, String> {
        let val = expression
            .parse::<f64>()
            .map_err(|_| format!("Cannot parse expression: {}", expression))?;
        Ok(Self {
            name: name.to_string(),
            expression: expression.to_string(),
            value_in_base: current_unit.to_base(val),
        })
    }
}
