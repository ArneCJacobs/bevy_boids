use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_inspector_egui::WorldInspectorPlugin;
use smooth_bevy_cameras::{controllers::fps::{FpsCameraBundle, FpsCameraController, FpsCameraPlugin}, LookTransformPlugin};
use bevy::render::primitives::Aabb;
use bevy_prototype_debug_lines::*;
use bevy::render::mesh::{self, PrimitiveTopology};
use std::f32::consts::PI;
use bevy::math::const_vec3;
use itertools::Itertools;

struct Field(Aabb);

fn main() {
    let (width, height, length) = (50.0, 50.0, 50.0);
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(FpsCameraPlugin::default())
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(LookTransformPlugin)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .insert_resource(RapierConfiguration{
            gravity: Vec3::new(0.0, 0.0, 0.0),
            ..Default::default()
        })
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup)
        .add_system(cursor_grab_system)
        .add_system(draw_field_system)
        .add_system(debug_draw_boid)
        .insert_resource(Field(Aabb::from_min_max(
                    Vec3::new(-width/2.0, -height/2.0, -length/2.0),
                    Vec3::new(width/2.0, height/2.0, length/2.0)
        )))
        //.add_startup_system(setup)
        .run();
}


fn debug_draw_boid(
    mut lines: ResMut<DebugLines>,
    boids: Query<(&Transform, &Velocity, &ExternalImpulse), With<Boid>>
) {
    for (transform, velocity, external_impluse) in boids.iter() {
        let pos = transform.translation;
        lines.line_colored(
            pos,
            pos + velocity.linvel.normalize(),
            0.0,
            Color::GREEN
        );

        lines.line_colored(
            pos,
            pos + external_impluse.impulse.normalize(),
            0.0,
            Color::RED
        );
    }

}
// hides mouse
pub fn cursor_grab_system(
    mut windows: ResMut<Windows>,
    btn: Res<Input<MouseButton>>,
    key: Res<Input<KeyCode>>,
    mut camera_controller_query: Query<&mut FpsCameraController>,
) {
    let window = windows.get_primary_mut().unwrap();

    let mut camera_controller = camera_controller_query.get_single_mut().unwrap();

    if btn.just_pressed(MouseButton::Left) {
        window.set_cursor_lock_mode(true);
        window.set_cursor_visibility(false);
        camera_controller.enabled = true;
    }

    if key.just_pressed(KeyCode::Escape) {
        window.set_cursor_lock_mode(false);
        window.set_cursor_visibility(true);
        camera_controller.enabled = false;
    }
}


fn setup_graphics(mut commands: Commands) {

    let controller = FpsCameraController {
        enabled: false,
        smoothing_weight : 0.6,
        translate_sensitivity: 0.1,
        mouse_rotate_sensitivity: Vec2::splat(0.001),
        ..Default::default()
    };
    commands
        .spawn_bundle(PerspectiveCameraBundle::default())
        .insert_bundle(FpsCameraBundle::new(
            controller,
            PerspectiveCameraBundle {
                transform: Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
                ..Default::default()
            },
            Vec3::new(-3.0, 3.0, 10.0),
            Vec3::new(0., 0., 0.),
        ));
}

//fn draw_bounding_box_system(
    //mut lines: ResMut<DebugLines>,
    //query: Query<&bevy::render::primitives::Aabb>
//) {
    //for aabb in query.iter() {
        //draw_bounding_box(&mut lines, aabb);
    //}

//}
//
fn draw_field_system(
    mut lines: ResMut<DebugLines>,
    field: Res<Field>,
) {
    draw_bounding_box(&mut lines, &field.0);
}

pub fn to_bvec3(bitmask: u8) -> BVec3 {
    BVec3::new(
        (bitmask & 0b100) != 0,
        (bitmask & 0b010) != 0,
        (bitmask & 0b001) != 0,
    )
}

pub fn draw_bounding_box(lines: &mut ResMut<DebugLines>, aabb: &Aabb) {
    let min = aabb.min().into();
    let max = aabb.max().into();

    let connections = [
        (0b000, 0b100),
        (0b000, 0b010),
        (0b000, 0b001),

        (0b100, 0b110),
        (0b100, 0b101),

        (0b010, 0b110),
        (0b010, 0b011),

        (0b001, 0b101),
        (0b001, 0b011),

        (0b011, 0b111),
        (0b101, 0b111),
        (0b110, 0b111),
    ];

    for (from, to) in connections {
        lines.line_colored(
            Vec3::select(to_bvec3(from), min, max),
            Vec3::select(to_bvec3(to), min, max),
            0.0,
            Color::GREEN
        );
    }
}

const FORWARD: Vec3 = const_vec3!([0.0, 0.0, -1.0]);
const UP: Vec3 = Vec3::Y;
const LEFT: Vec3 = Vec3::X;


fn create_boid_bundle(
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) -> BoidBundle {

    let a1 = (2.0 * PI) * 0.0/3.0; 
    let a2 = (2.0 * PI) * 1.0/3.0; 
    let a3 = (2.0 * PI) * 2.0/3.0; 

    let vertices: [([f32; 3], [f32; 2]); 4] = [
        ((2.0 * FORWARD).into(), [1.0, 1.0]),
        ((a1.sin() * LEFT + a1.cos() * UP).into(), [1.0, 1.0]),
        ((a2.sin() * LEFT + a2.cos() * UP).into(), [1.0, 1.0]),
        ((a3.sin() * LEFT + a3.cos() * UP).into(), [1.0, 1.0]),
    ];

    let indices_temp = vec![
        0, 1, 2,
        0, 2, 3,
        0, 3, 1,
        1, 3, 2,
    ];
    let indices = mesh::Indices::U32(indices_temp.clone());

    let mut positions = Vec::new();
    //let mut normals = Vec::new();
    let mut uvs = Vec::new();
    for (position,uv) in vertices.iter() {
        positions.push(*position);
        //normals.push(*normal);
        uvs.push(*uv);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_indices(Some(indices));
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.clone());
    //mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.duplicate_vertices();
    mesh.compute_flat_normals();

    // add entities to the world
    let pbr_bundle =  PbrBundle {
        mesh: meshes.add(mesh),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    };

    let mut chunks = vec![];
    for chunk in indices_temp.chunks_exact(3) {
        chunks.push([chunk[0], chunk[1],chunk[2]]);
    }

    let collider = Collider::convex_mesh(
        positions.into_iter().map(|v| v.into()).collect(),
        chunks.as_slice(),
    ).unwrap();

    let mass_properties = MassProperties {
        mass: 10.0,
        ..Default::default()
    };

    let external_impluse = ExternalImpulse {
        impulse: Vec3::new(0.0, 0.0, 0.0),
        ..Default::default()
    };

    return BoidBundle {
        name: Name::new("Boid"),
        pbr_bundle,
        collider,
        mass_properties,
        rigid_body: RigidBody::Dynamic,
        external_impluse,
        boid: Boid,
        velocity: Velocity::default()
    };

}

#[derive(Component)]
struct Boid;

#[derive(Bundle)]
struct BoidBundle {
    name: Name,
    #[bundle]
    pbr_bundle: PbrBundle,
    collider: Collider,
    rigid_body: RigidBody,
    mass_properties: MassProperties,
    external_impluse: ExternalImpulse,
    boid: Boid,
    velocity: Velocity,
}

fn setup(
    mut commands: Commands,
    meshes: ResMut<Assets<Mesh>>,
    materials: ResMut<Assets<StandardMaterial>>,
    ) {
    let boid_bundle = create_boid_bundle(meshes, materials);
    commands.spawn_bundle(boid_bundle);
    // light
    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
}
