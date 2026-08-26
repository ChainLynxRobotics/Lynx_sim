use jni::{
    EnvUnowned,
    errors::{Error, ThrowRuntimeExAndDefault},
    jni_mangle,
    objects::JClass,
    sys::jlong,
};

use crate::{field::Field, physics_world::PhysicsWorld, robot::Robot};

pub struct World {
    pub physics_world: PhysicsWorld,
    pub robots: Vec<Robot<4>>,
    pub field: Option<*mut Box<dyn Field>>,
}
#[jni_mangle("com.chainlynx8248.Lynx_sim.World")]
pub fn create_fieldless_world<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
) -> jlong {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        Ok(Box::into_raw(Box::new(World {
            physics_world: PhysicsWorld::default(),
            robots: Vec::new(),
            field: None,
        })) as jlong)
    });
    outcome.resolve::<ThrowRuntimeExAndDefault>()
}
#[jni_mangle("com.chainlynx8248.Lynx_sim.World")]
pub unsafe fn set_field<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    world_ptr: jlong,
    field_ptr: jlong,
) {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        let world = unsafe { &mut *(world_ptr as *mut World) };
        let field = field_ptr as *mut Box<dyn Field>;
        world.field = Some(field);
        Ok(())
    });
    outcome.resolve::<ThrowRuntimeExAndDefault>()
}
