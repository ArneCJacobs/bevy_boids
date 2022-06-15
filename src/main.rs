use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use smooth_bevy_cameras::{controllers::fps::{FpsCameraBundle, FpsCameraController, FpsCameraPlugin}, LookTransformPlugin};
use bevy::render::primitives::Aabb;
use bevy_prototype_debug_lines::*;
use bevy::render::mesh::{self, PrimitiveTopology};
use std::f32::consts::PI;
use bevy::math::const_vec3;
use itertools::Itertools;
use rand::distributions::Uniform;
use rand::{Rng, thread_rng};
use bevy_rapier3d::math::Rot;
use core::ops::Deref;
use vec_map::VecMap;
use bevy_inspector_egui::{WorldInspectorPlugin, Inspectable, RegisterInspectable};
use bevy_inspector_egui::prelude::*;
use noise::{NoiseFn, OpenSimplex};

struct Field(Aabb);

struct SpacialMap { 
    map: VecMap<Vec<Entity>>,
    min_chunk_pos: Option<IVec3>,
    size_vec: Option<IVec3>,
}


#[derive(Component, Clone, Inspectable)]
struct ChunkPos{ 
    pos: IVec3,
}

impl ChunkPos {
    fn new(pos: IVec3) -> Self {
        ChunkPos {
            pos,
        }
    }
}

const PXS: [Vec3; 7] = [
    Vec3::ZERO, 
    Vec3::X, 
    const_vec3!([-1., 0., 0.]),
    Vec3::Y, 
    const_vec3!([0., -1., 0.]),
    Vec3::Z, 
    const_vec3!([0., 0., -1.]),
];

impl Default for SpacialMap {
    fn default() -> Self {
        SpacialMap {
            map: VecMap::new(),
            min_chunk_pos: None,
            size_vec: None, // x: num rows, y: num cols, z: num tables
        }
    }
}

impl SpacialMap {
    fn init(&mut self, min_chunk_pos: IVec3, size_vec: IVec3) {
        self.min_chunk_pos = Some(min_chunk_pos);
        self.size_vec = Some(
            IVec3::new(
                1,
                size_vec.x,
                size_vec.x * size_vec.y,
            )
        );
    }

    fn chunk_pos_to_key_unchecked(&self, chunk_pos: IVec3) -> usize {
        let temp = chunk_pos - self.min_chunk_pos.unwrap();
        return temp.dot(self.size_vec.unwrap()) as usize;
    }

    fn replace_chunk(&mut self, chunk_pos: IVec3, boids: Vec<Entity>) {
        let key = self.chunk_pos_to_key_unchecked(chunk_pos);
        self.map.insert(key,boids);
    }

    fn get_chunk(&self, chunk_pos: IVec3) -> Option<&Vec<Entity>> {
        let key = self.chunk_pos_to_key_unchecked(chunk_pos);
        self.map.get(key)
    }

    fn get_chunk_mut(&mut self, chunk_pos: IVec3) -> Option<&mut Vec<Entity>> {
        let key = self.chunk_pos_to_key_unchecked(chunk_pos);
        self.map.get_mut(key)
    }

    fn get_boids_in_chunk_in_range<'a>(
        &'a self, 
        boid_settings: &'a Res<BoidSettings>, 
        field: &'a Res<Field>, 
        pos: Vec3
    ) -> impl Iterator<Item=&Entity> + 'a {
        //let chunk_pos = to_chunk_location(boid_settings, pos);
        let min: Vec3 = field.0.min().into();
        let max: Vec3 = field.0.max().into();
        //println!("====================");
        //println!("{:?}", pos);
        PXS.iter()
            .map(move |&px| pos + px * boid_settings.view_radius)
            .map(move |new_pos| keep_pos_inside_field(min, max, new_pos) )
            .map(move |new_pos| to_chunk_location(&boid_settings, new_pos))
            //.map(|chunk_pos| {println!("{:?}", chunk_pos); chunk_pos})
            .map(|chunk_pos| self.chunk_pos_to_key_unchecked(chunk_pos))
            .unique()
            //.map(|chunk_pos| {println!("{:?}", chunk_pos); chunk_pos})
            .filter_map(|chunk_key| self.map.get(chunk_key))
            .flat_map(|vec| vec.iter())
    }
}


fn main() {
    let field_width = 400.0;
    let field = Vec3::splat(field_width/2.0);
    App::new()
        .add_plugins(DefaultPlugins)
        //.add_plugin(WorldInspectorPlugin::new())
        .add_plugin(DebugLinesPlugin::default())
        // rapier
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .insert_resource(RapierConfiguration{
            gravity: Vec3::new(0.0, 0.0, 0.0),
            ..Default::default()
        })
        //.add_plugin(RapierDebugRenderPlugin::default())
        // basic setup
        .add_plugin(FpsCameraPlugin::default())
        .add_plugin(LookTransformPlugin)
        .add_startup_system(setup_graphics)
        .add_system(cursor_grab_system)
        .add_system(draw_field_system)
        //.add_system(debug_draw_boid)
        // boids
        .insert_resource(Field(Aabb::from_min_max(-field, field)))
        //.insert_resource(BoidSettings::default())
        .add_plugin(InspectorPlugin::<BoidSettings>::new())
        .add_plugin(InspectorPlugin::<WindSettings>::new())
        .add_startup_system(spawn_boids.before(init_spatial_map))
        .add_system(update_boids)
        .add_system(keep_inside_field.after(update_boids))
        .add_system(boid_look_at_velocity.after(update_boids))
        .add_system(enforce_min_speed.after(update_boids))
        // spacial map
        .insert_resource(SpacialMap::default())
        .add_startup_system(init_spatial_map.after(spawn_boids))
        .add_system(update_spacial_map.after(keep_inside_field))
        //.add_system(print_spacial_map.before(update_boids))
        .register_inspectable::<ChunkPos>()
        .add_system(windfield_system)
        .run();
}


#[derive(Inspectable)]
struct WindSettings {
    strength: f32,
}

impl Default for WindSettings {
    fn default() -> Self {
        WindSettings {
            strength: 1.
        }
    }
}

fn windfield_system(
    mut entities: Query<(&Transform, &mut ExternalForce)>,
    wind_settings: Res<WindSettings>
) {
    let simplex = OpenSimplex::new();
    for (transform, mut external_force) in entities.iter_mut() {
        let pos1: [f64; 3] = transform.translation.as_dvec3().into();
        let pos2: [f64; 3] = (transform.translation + Vec3::new(2., 5., 0.)).as_dvec3().into();
        let pos3: [f64; 3] = (transform.translation + Vec3::new(9., -5., -25.)).as_dvec3().into();
        let turbulence = Vec3::new(
            simplex.get(pos1) as f32,
            simplex.get(pos3) as f32,
            simplex.get(pos3) as f32,
        );
        external_force.force += turbulence * wind_settings.strength;
    }
}

fn print_spacial_map(
    spacial_map: Res<SpacialMap>
) {
    println!("====================");
    println!("{:?}", spacial_map.map);
}

fn init_spatial_map(
    field: Res<Field>,
    boid_settings: Res<BoidSettings>,
    mut spacial_map: ResMut<SpacialMap>,
    boids: Query<(Entity, &Transform), With<Boid>>,
    mut commands: Commands,
) {
    let min: Vec3 = field.0.min().into();
    let max: Vec3 = field.0.max().into();
    let mut map = boids.iter().into_group_map_by(|(_, pos)| to_chunk_location(&boid_settings, pos.translation));

    let temp_vec = ((max - min)/boid_settings.view_radius).ceil().as_ivec3();
    spacial_map.init(
        to_chunk_location(&boid_settings, min), 
        temp_vec, 
    );

    spacial_map.map = VecMap::with_capacity(temp_vec.dot(IVec3::ONE) as usize);
    //spacial_map.map.reserve_len(temp_vec.dot(IVec3::ONE) as usize);
    for (chunk_pos, boids) in map.drain() {
        for (boid, _) in boids.iter(){
            commands.entity(*boid)
                .insert(ChunkPos::new(chunk_pos));
        }
        
        spacial_map.replace_chunk(
            chunk_pos, 
            boids.iter().map(|(entity, _)| entity.clone()).collect()
        );
    }
}

fn update_spacial_map(
    boid_settings: Res<BoidSettings>,
    mut spacial_map: ResMut<SpacialMap>,
    mut boids: Query<(Entity, &Transform, Option<&mut ChunkPos>), (With<Boid>, Changed<Transform>)>,
    mut commands: Commands,
) {
    for (entity, transform, chunk_pos_opt) in boids.iter_mut() {
        let new_chunk_pos = to_chunk_location(&boid_settings, transform.translation);

        if let Some(mut chunk_pos) = chunk_pos_opt {
            if new_chunk_pos != chunk_pos.pos {
                //let new_chunk = spacial_map.get_chunk_mut(new_chunk_pos).unwrap();
                //new_chunk.push(entity);
                let chunk = spacial_map.get_chunk_mut(chunk_pos.pos).unwrap();
                let index = chunk.iter().position(|&e| e == entity).unwrap();
                chunk.remove(index);
                chunk_pos.pos = new_chunk_pos; 

                if let Some(new_chunk) = spacial_map.get_chunk_mut(new_chunk_pos) {
                    new_chunk.push(entity);
                } else {
                    spacial_map.replace_chunk(new_chunk_pos, vec![entity]);
                }
            }
        } else {
            commands.entity(entity)
                .insert(ChunkPos{ pos: new_chunk_pos });
            if let Some(new_chunk) = spacial_map.get_chunk_mut(new_chunk_pos) {
                new_chunk.push(entity);
            } else {
                spacial_map.replace_chunk(new_chunk_pos, vec![entity]);
            }
        } 

    }

    //for (entity, new_chunk_pos) in itertools::zip(test, test2) {
        //commands.entity(entity)
            //.insert(new_chunk_pos);
    //}

}

fn to_chunk_location(boid_settings: &Res<BoidSettings>, pos: Vec3) -> IVec3 {
    (pos / boid_settings.view_radius).floor().as_ivec3()
}

fn enforce_min_speed(
    boid_settings: Res<BoidSettings>,
    mut boids: Query<&mut Velocity, With<Boid>>
) {
    for mut velocity in boids.iter_mut() {
        if velocity.linvel.length_squared() == 0.0 {
            velocity.linvel = Vec3::new(0.1, 0.0, 0.0);   
        } 
        velocity.linvel = velocity.linvel.clamp_length_min(boid_settings.min_speed);
    }
}

#[derive(Inspectable)]
struct BoidSettings {
    #[inspectable(min = 0.0)]
    view_radius: f32,
    #[inspectable(min = 0.0)]
    avoid_raduis: f32,
    #[inspectable(min = 0.0)]
    max_speed: f32,
    #[inspectable(min = 0.0)]
    min_speed: f32,
    max_steer_force: f32,

    #[inspectable(min = 0.0, max = 10.0)]
    alignment_weight: f32,
    #[inspectable(min = 0.0, max = 10.0)]
    cohesion_weight: f32,
    #[inspectable(min = 0.0, max = 10.0)]
    seperation_weight: f32,
    boid_count: u64,
}

impl Default for BoidSettings {
    fn default() -> Self {
        BoidSettings {
            alignment_weight: 1.0,
            cohesion_weight: 0.6,
            seperation_weight: 1.0,

            max_steer_force: 3.0,
            min_speed: 15.0,
            max_speed: 40.0,

            view_radius: 20.,
            avoid_raduis: 5.,

            boid_count: 2000,
        }
    }

}

fn update_boids(
    mut boids: Query<(Entity, &Transform, &mut ExternalForce, &Velocity), With<Boid>>,
    boid_settings: Res<BoidSettings>,
    children: Query<&Parent>,
    other_boids_transform: Query<&Transform, With<Boid>>,
    rapier_context: Res<RapierContext>,
    mut lines: ResMut<DebugLines>,
    spacial_map: Res<SpacialMap>,
    field: Res<Field>
) {
    let square_view_radius = boid_settings.view_radius * boid_settings.view_radius;
    let square_avoid_radius = boid_settings.avoid_raduis * boid_settings.avoid_raduis;

    let boid_data = boids.iter().map(|(_, boid_transform, _, _)| {
        let mut flock_heading = Vec3::ZERO;
        let mut flock_center = Vec3::ZERO;
        let mut seperation_heading = Vec3::ZERO;
        let mut flock_size = 0;
        for other_boid in spacial_map.get_boids_in_chunk_in_range(&boid_settings, &field, boid_transform.translation) {
            let other_boid_transform = other_boids_transform.get(*other_boid).unwrap();

            if boid_transform == other_boid_transform {
                continue;
            }

            //lines.line_colored(boid_transform.translation, other_boid_transform.translation, 0., Color::GREEN);

            let offset = other_boid_transform.translation - boid_transform.translation;
            let square_dist = offset.length_squared();
            if square_dist < square_view_radius {
                flock_size += 1; 
                flock_heading += other_boid_transform.forward();
                flock_center += other_boid_transform.translation;
                if square_dist < square_avoid_radius {
                    seperation_heading -= offset / square_dist;
                }
            }
        }

        //for (e1, e2, b) in rapier_context.intersections_with(entity) {
        //}
        return (flock_heading, flock_center, seperation_heading, flock_size);
    }).collect::<Vec<_>>();

    for (boid_cur_data, boid_calc_data) in itertools::zip(boids.iter_mut(), boid_data) {
        let (entity, transform, mut external_force, velocity) = boid_cur_data;
        let (flock_heading, flock_center, seperation_heading, flock_size) = boid_calc_data;
        external_force.force = Vec3::ZERO;
        if flock_size == 0 {
            continue;
        }
        let avg_flock_center = flock_center / (flock_size as f32);
        let flock_center_offset = avg_flock_center - transform.translation;

        let alignment_force = steer_towards(velocity.linvel, flock_heading, &boid_settings) * boid_settings.alignment_weight;
        let collision_avoidence_force = steer_towards(velocity.linvel, flock_center_offset, &boid_settings) * boid_settings.cohesion_weight;
        let seperation_force = steer_towards(velocity.linvel, seperation_heading, &boid_settings) * boid_settings.seperation_weight;

        external_force.force += alignment_force;
        external_force.force += collision_avoidence_force;
        external_force.force += seperation_force;
    }

}

fn steer_towards(velocity: Vec3, vector: Vec3, boid_settings: &Res<BoidSettings>) -> Vec3 {
    if vector.length_squared() < 0.1 {
        return Vec3::ZERO;
    }
    let new_vector = vector.normalize() * boid_settings.max_speed - velocity;
    return new_vector.clamp_length_max(boid_settings.max_steer_force);
}

fn keep_pos_inside_field(min: Vec3, max: Vec3, pos: Vec3) -> Vec3 {
    let new_pos = Vec3::select(min.cmplt(pos), pos, max);
    Vec3::select(new_pos.cmple(max), new_pos, min)
}

fn keep_inside_field(
    field: Res<Field>,
    mut boids: Query<&mut Transform, With<Boid>>) {
    let min: Vec3 = field.0.min().into();
    let max: Vec3 = field.0.max().into();
    for mut transform in boids.iter_mut() {
        transform.translation = keep_pos_inside_field(min, max, transform.translation);
    }
    
}

fn boid_look_at_velocity(mut boids: Query<(&mut Transform, &Velocity), With<Boid>>) {
    for (mut transform, velocity) in boids.iter_mut() {
        if velocity.linvel.length_squared() > 0.001 {
            let pos = transform.translation;
            let up = transform.up();
            transform.look_at(
                pos + velocity.linvel.normalize(),
                up
            );

        }
    }

}

fn debug_draw_boid(
    mut lines: ResMut<DebugLines>,
    boids: Query<(&Transform, &Velocity, &ExternalForce), With<Boid>>
) {
    for (transform, velocity, external_force) in boids.iter() {
        let pos = transform.translation;
        lines.line_colored(
            pos,
            pos + velocity.linvel.normalize(),
            0.0,
            Color::GREEN
        );

        lines.line_colored(
            pos,
            pos + external_force.force.normalize(),
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

    // light
    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    });

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
            Vec3::new(0., 0., 300.0),
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
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
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
        ..Default::default()
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

    let external_force = ExternalForce {
        force: Vec3::new(0.0, 0.0, 0.0),
        ..Default::default()
    };

    return BoidBundle {
        name: Name::new("Boid"),
        pbr_bundle,
        collider,
        mass_properties,
        rigid_body: RigidBody::Dynamic,
        external_force,
        boid: Boid,
        velocity: Velocity::default(),
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
    external_force: ExternalForce,
    boid: Boid,
    velocity: Velocity,
}

fn spawn_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    field: Res<Field>,
    boid_settings: Res<BoidSettings>,
) {
    let mut rng = thread_rng();
    let radius = f32::min(
        field.0.min().min_element(),
        field.0.max().min_element()
    ).abs();
    let distribution = Uniform::new(-radius, radius);
    let rotation_distribution = Uniform::new(-1., 1.);
    for _ in 0..boid_settings.boid_count {
        let boid_bundle = create_boid_bundle(&mut meshes, &mut materials);

        let linvel = Vec3::new(
            rng.sample(rotation_distribution),
            rng.sample(rotation_distribution),
            rng.sample(rotation_distribution),
        );
        let velocity = Velocity {
            linvel,
            ..Default::default()
        };

        let translation = Vec3::new(
            rng.sample(distribution),
            rng.sample(distribution),
            rng.sample(distribution),
        );


        let transform = Transform::from_translation(translation)
            .with_scale(Vec3::splat(0.5));


        commands.spawn_bundle(boid_bundle)
            .insert(velocity)
            .insert(transform);
            //.with_children(|children| {
                //children.spawn()
                    //.insert(Collider::ball(boid_settings.view_radius))
                    //.insert(Sensor(true));
            //});
    }
}
