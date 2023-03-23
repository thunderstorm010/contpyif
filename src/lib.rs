use std::io::{BufReader, Read};

use pyo3::prelude::*;
use rosrust_msg::calrov_msgs::manual_control;
use serial::{SystemPort, SerialPort};

fn read_adc(first_byte: u8, second_byte: u8) -> u16 {
    let last_bit = (first_byte >> 0) & 1;
    let second_byte = second_byte ^ (last_bit ^ second_byte) & (1 << 7);
    let first_byte = first_byte >> 1;
    u16::from_be_bytes([first_byte, second_byte])
}

#[derive(Clone, Copy)]
#[pyclass]
pub enum RotaryData {
    Decrease,
    Increase,
    None
}


impl From<u8> for RotaryData {
    fn from(value: u8) -> Self {
        if (value & 0b01000000) == 1 {
            Self::Increase
        } else if (value & 0b00100000) == 1 {
            Self::Decrease
        } else {
            Self::None
        }
    }
}

impl Into<i8> for RotaryData {
    fn into(self) -> i8 {
        match self {
            RotaryData::Decrease => -1,
            RotaryData::Increase => 1,
            RotaryData::None => 0,
        }
    }
}

#[pyclass]
#[derive(Clone, Copy)]
pub struct ControllerRead {
    #[pyo3(get)]
    pub x: u16,
    #[pyo3(get)]
    pub y: u16,
    #[pyo3(get)]
    pub z: u16,
    #[pyo3(get)]
    pub r: RotaryData,
    #[pyo3(get)]
    pub buttons: (u8, u8),
    #[pyo3(get)]
    pub gain: u16
}

#[pyclass]
struct Controller {
    port: BufReader<SystemPort>
}

#[pymethods]
impl Controller {
    #[new]
    fn new() -> Self {
        let mut serial = serial::open("/dev/ttyACM0").unwrap();
        serial.reconfigure(& |settings| {
            settings.set_baud_rate(serial::BaudRate::Baud9600)?;
            Ok(())
        }).unwrap();

        let buf = BufReader::with_capacity(32, serial);
        
        Controller {
            port: buf
        }
    }

    fn read(&mut self) -> PyResult<ControllerRead> {
        // Loop reading bytes one-by-one
        loop {
            let mut data = [0; 1];
            self.port.read_exact(&mut data)?;
            let byte = data[0];
            // F != 1
            if (byte & 0b10000000) != 1 {
                continue;
            }   
            // read the actual message..
            let mut data = [0; 10];
            self.port.read_exact(&mut data)?;
            let buttons = (data[0], data[1]);
            let gain = read_adc(data[8], data[9]);
            let y = read_adc(data[4], data[5]);
            let x = read_adc(data[2], data[3]);
            let z = read_adc(data[6], data[7]);
            let rot =  RotaryData::from(data[3]);
            return Ok(ControllerRead {
                x, y, z, r: rot, buttons, gain
            })
        }
    }
}

fn map_range(input: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i32 {
    out_min + ((out_max - out_min) / (in_max - in_min)) * (input - in_min)
}


fn to_manual_control(x: u16, y: u16, z: u16, r: i16, g: u16, mut b: (u8, u8)) -> (i16, i16, i16, i16, u16) {
    if (b.0 & 0b00000001) == 1 {
        b.1 |= 0b10000000
    }
    b.0 >>= 1;

    let x = map_range((x * g) as i32, 0, 4095 * 4095, -1000, 1000);
    let y = map_range((y * g) as i32, 0, 4095 * 4095, -1000, 1000);
    let z = map_range((z * g) as i32, 0, 4095 * 4095, -1000, 1000);
    let r = map_range((r as u16 * g) as i32, 0, 2000 * 4095, -1000, 1000);
    let b = u16::from_be_bytes([b.0, b.1]);
    (x as _, y as _, z as _, r as _, b as _)
}

#[pyclass]
#[derive(Clone)]
pub struct ControllerSend {
    publisher: rosrust::Publisher<rosrust_msg::calrov_msgs::manual_control>,
    pub yaw: i16
}

#[derive(Clone, Copy)]
#[pyclass]
pub struct ManualControl {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub yaw: i16,
    pub buttons: i16
}

#[pymethods]
impl ControllerSend {
    #[new]
    pub fn new() -> Self {
        ControllerSend { publisher: rosrust::publish("MANUAL_CONTROL", 100).unwrap(), yaw: 0 }
    }

    pub fn send(&mut self, read: ControllerRead) -> ManualControl {
        self.yaw += <RotaryData as Into<i8>>::into(read.r) as i16 * 10;
        let mc = to_manual_control(read.x, read.y, read.z, self.yaw + 1000, read.gain, read.buttons);
        let message = manual_control {
            x: mc.0,
            y: mc.1,
            z: mc.2,
            yaw: self.yaw,
            buttons: mc.4 as i16
        };
        self.publisher.send(message).unwrap();
        ManualControl { x: mc.0, y: mc.1, z: mc.2, yaw: self.yaw, buttons: mc.4 as i16 }
    }
}

#[pyfunction]
fn ros_init() {
    rosrust::init("ros_controller");
}

/// A Python module implemented in Rust.
#[pymodule]
fn contpyif(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<RotaryData>()?;
    m.add_class::<ControllerRead>()?;
    m.add_class::<ControllerSend>()?;
    m.add_class::<ManualControl>()?;
    m.add_class::<Controller>()?;

    m.add_function(wrap_pyfunction!(ros_init, m)?)?;
    Ok(())
}