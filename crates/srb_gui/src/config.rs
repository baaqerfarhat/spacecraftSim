use serde::{Deserialize, Serialize};
use std::fmt::Display;

#[derive(
    Deserialize, Serialize, display_json::DebugAsJson, display_json::DisplayAsJson, Clone, PartialEq,
)]
pub struct TaskConfig {
    pub task: Task,
    pub seed: u64,
    pub num_envs: u64,
    pub domain: Scenario,
    pub robot: String,
    pub enable_ui: bool,
}

impl Default for TaskConfig {
    fn default() -> Self {
        Self {
            task: Task::SampleCollection,
            seed: 0,
            num_envs: 1,
            domain: Scenario::Moon,
            robot: "dataset".to_owned(),
            enable_ui: false,
        }
    }
}

impl TaskConfig {
    pub fn set_exec_env(&self, mut exec: subprocess::Exec) -> subprocess::Exec {
        if self.task == Task::LocomotionVelocityTracking {
            exec = exec.args(&["agent", "rand"]);
            if !self.enable_ui {
                exec = exec.arg("--hide_ui");
            }
        } else {
            exec = exec.args(&[
                "agent",
                "teleop",
                "--teleop_device",
                "keyboard",
                "spacemouse",
                "ros",
                "haptic",
            ]);
            if self.enable_ui {
                exec = exec.args(&["--interface", "gui", "ros"]);
            } else {
                exec = exec.args(&["--hide_ui", "--interface", "gui"]);
            }
        }
        exec = exec.args(&["--task", self.task.to_string().trim_matches('"')]);
        exec = exec.arg(format!("env.scene.num_envs={}", self.num_envs.max(1)));

        // Environment variables - Environment
        exec = exec.arg(format!(
            "env.seed={}",
            self.seed.to_string().trim_matches('"').to_owned()
        ));
        exec = exec.arg(format!(
            "env.domain={}",
            self.domain.to_string().trim_matches('"').to_owned()
        ));
        exec = exec.arg(format!("env.robot={}", self.robot.trim_matches('"')));

        // Environment variables - GUI
        exec = exec.env(
            "DISPLAY",
            std::env::var("SRB_DISPLAY").unwrap_or(":0".to_string()),
        );

        // Environment variables - ROS
        exec = exec.env(
            "ROS_DOMAIN_ID",
            std::env::var("ROS_DOMAIN_ID").unwrap_or("0".to_string()),
        );
        exec = exec.env(
            "RMW_IMPLEMENTATION",
            std::env::var("RMW_IMPLEMENTATION").unwrap_or("rmw_cyclonedds_cpp".to_string()),
        );

        exec
    }
}

#[derive(
    Deserialize,
    Serialize,
    Debug,
    display_json::DisplayAsJson,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Hash,
    Default,
    strum::EnumIter,
)]
#[serde(rename_all = "snake_case")]
// TODO[low]: Automatically extract list of tasks from file at py:srb.utils.path.SRB_ENV_CACHE_PATH
pub enum Task {
    _Aerial,
    _Ground,
    _Manipulation,
    _Orbital,
    DebrisCapture,
    Excavation,
    LocomotionVelocityTracking,
    MobileDebrisCapture,
    OrbitalEvasion,
    PegInHoleAssembly,
    PegInHoleAssemblyMulti,
    #[default]
    SampleCollection,
    SampleCollectionMulti,
    SolarPanelAssembly,
}

#[derive(Deserialize, Serialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Scenario {
    #[serde(alias = "asteroid")]
    Asteroid,
    #[serde(alias = "earth")]
    Earth,
    #[serde(alias = "mars")]
    Mars,
    #[default]
    #[serde(alias = "moon")]
    Moon,
    #[serde(alias = "orbit")]
    Orbit,
}

impl Display for Scenario {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Scenario::Asteroid => write!(f, "asteroid"),
            Scenario::Earth => write!(f, "earth"),
            Scenario::Mars => write!(f, "mars"),
            Scenario::Moon => write!(f, "moon"),
            Scenario::Orbit => write!(f, "orbit"),
        }
    }
}

impl Scenario {
    /// Magnitude of gravitational acceleration in m/s².
    ///
    /// # Assumptions
    ///
    /// - Asteroid: 50% gravitational acceleration of Ceres (largest body in the asteroid belt).
    /// - Orbit: No gravitational acceleration.
    #[must_use]
    pub fn gravity_magnitude(self) -> f64 {
        match self {
            Self::Asteroid => 0.14219,
            Self::Earth => 9.80665,
            Self::Mars => 3.72076,
            Self::Moon => 1.62496,
            Self::Orbit => 0.0,
        }
    }
}
