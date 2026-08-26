use jni::{
    EnvUnowned,
    errors::{Error, ThrowRuntimeExAndDefault},
    jni_mangle,
    objects::JClass,
    sys::jlong,
};

use crate::{
    field::{Field, rebuilt::RebuiltField},
    jni::world::World,
};

pub struct RebuiltJNI {
    field: RebuiltField,
    world: *mut World,
}
#[jni_mangle("com.chainlynx8248.Lynx_sim.Rebuilt")]
pub unsafe fn new_field<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    world_ptr: jlong,
) -> jlong {
    let output = unowned_env.with_env(|_env| -> Result<_, Error> {
        let world_ptr = world_ptr as *mut World;
        let world = unsafe { &mut *(world_ptr) };
        Ok(Box::into_raw(Box::new(RebuiltJNI {
            field: RebuiltField::new(&mut world.physics_world),
            world: world_ptr,
        })) as jlong)
    });
    output.resolve::<ThrowRuntimeExAndDefault>()
}
#[jni_mangle("com.chainlynx8248.Lynx_sim.Rebuilt")]
pub unsafe fn get_field_trait<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    ptr: jlong,
) -> jlong {
    let output = unowned_env.with_env(|_env| -> Result<_, Error> {
        let world = unsafe { &mut *(ptr as *mut RebuiltJNI) };
        let a: &dyn Field = &world.field;
        let a = Box::into_raw(Box::new(Box::new(a))) as jlong;
        Ok(a)
    });
    output.resolve::<ThrowRuntimeExAndDefault>()
}
