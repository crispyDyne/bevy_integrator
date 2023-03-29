use std::f32::consts::PI;

use bevy::prelude::*;

pub fn build_environment(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    // add ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 0.2,
    });

    // add point light
    commands.spawn(PointLightBundle {
        transform: Transform::from_xyz(1.0, -5.0, 3.0),
        point_light: PointLight {
            intensity: 1600.0, // lumens - roughly a 100W non-halogen incandescent bulb
            color: Color::WHITE,
            shadow_depth_bias: 0.1,
            shadow_normal_bias: 0.9,
            shadows_enabled: true,
            ..default()
        },
        ..default()
    });

    // add ground plane
    let ground_transform = Transform::from_translation(Vec3 {
        x: 0.,
        y: 0.,
        z: 0.,
    }) * Transform::from_rotation(Quat::from_axis_angle(Vec3::X, PI / 2.));
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane {
            size: 1000.0,
            subdivisions: 10,
        })),
        material: materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 1.0,
            ..default()
        }),
        transform: ground_transform,
        ..default()
    });
}
