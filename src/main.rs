use safe_drive::{context::Context, error::DynError, logger::Logger, topic::subscriber};

use drobo_interfaces::msg::MdLibMsg;
use motor_controller::motor::Motor;
use std::sync::{Arc, Mutex};
use std::thread;

const MAX_PAWER_OUTPUT: f64 = 999.;
const FREQUENCY: f64 = 1000.;
const START_DUTY_CYCLE: f64 = 0.0;

fn main() -> Result<(), DynError> {
    // for debug
    let _logger = Logger::new("temporary_dmotor_ros");

    let ctx = Context::new()?;
    let node = ctx.create_node("temporary_dmotor_ros", None, Default::default())?;
    let subscriber = node.create_subscriber::<MdLibMsg>("md_driver_topic", None)?;
    let mut selector = ctx.create_selector()?;

    let setup_motors = [
        Motor::new(6, 3, START_DUTY_CYCLE, FREQUENCY),
        Motor::new(16, 1, START_DUTY_CYCLE, FREQUENCY),
        Motor::new(26, 2, START_DUTY_CYCLE, FREQUENCY),
        Motor::new(7, 4, START_DUTY_CYCLE, FREQUENCY),
        Motor::new(17, 5, START_DUTY_CYCLE, FREQUENCY),
        Motor::new(28, 8, START_DUTY_CYCLE, FREQUENCY),
    ];

    let mut motors = setup_motors
        .iter()
        .map(|pwm| pwm.duty_cycle.clone())
        .collect::<Vec<_>>();

    // start
    {
        for mut setup_motor in setup_motors {
            thread::spawn(move || {
                setup_motor.start();
            });
        }
    }

    selector.add_subscriber(subscriber, {
        Box::new(move |msg| {
            topic_callback(msg, &mut motors);
        })
    });

    loop {
        selector.wait()?;
    }
}

fn topic_callback(msg: subscriber::TakenMsg<MdLibMsg>, motors: &mut Vec<Arc<Mutex<f64>>>) {
    // for debug
    if msg.address < motors.len() as u8 {
        *motors[msg.address as usize].lock().unwrap() =
            (if msg.phase { 1. } else { -1. } * msg.power as f64) / MAX_PAWER_OUTPUT;
    }
}
