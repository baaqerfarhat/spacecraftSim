mod assets;
mod config;
mod enums;

pub use assets::*;
pub use config::*;
pub use enums::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "yaml")]
    fn extract() {
        figment::Jail::expect_with(|jail| {
            // Arrange
            let cfg_path = if cfg!(feature = "yaml") {
                jail.create_file(
                    "config.yaml",
                    r"
                    seed: 42
                    assets:
                      vehicle:
                        variant: dataset
                ",
                )?;
                Some("config.yaml")
            } else {
                None
            };
            jail.set_env("SRB_SCENARIO", "ORBIT");
            jail.set_env("SRB_ASSETS_TERRAIN_VARIANT", "PROCEDURAL");
            jail.set_env("SRB_DETAIL", "0.2");

            // Act
            let config = EnvironmentConfig::extract(cfg_path, Some("SRB_"), None)?;

            // Assert
            assert_eq!(
                config,
                EnvironmentConfig {
                    scenario: Scenario::Orbit,
                    assets: Assets {
                        vehicle: Asset {
                            variant: AssetVariant::Dataset,
                        },
                        terrain: Asset {
                            variant: AssetVariant::Procedural,
                        },
                        ..EnvironmentConfig::default().assets
                    },
                    seed: 42,
                    detail: 0.2,
                }
            );

            Ok(())
        });
    }
}
