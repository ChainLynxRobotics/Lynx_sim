use jni::{
    EnvUnowned,
    errors::Error,
    jni_mangle,
    objects::JClass,
    sys::{jint, jlong},
};

pub mod rebuilt;
pub mod world;

#[derive(Default)]
struct Counter {
    count: i32,
}
impl Counter {
    fn increment(&mut self) {
        self.count += 1;
    }
}

#[jni_mangle("com.chainlynx8248.Lynx_sim.VendorJNI")]
pub fn new_counter<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
) -> jlong {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        Ok(Box::into_raw(Box::new(Counter::default())) as jlong)
    });

    outcome.resolve::<jni::errors::ThrowRuntimeExAndDefault>()
}

#[jni_mangle("com.chainlynx8248.Lynx_sim.VendorJNI")]
pub unsafe fn increment_counter<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    ptr: jlong,
) {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        // SAFETY: We cant really do anything here we just have to trust that the java passes a valid pointer
        let counter = unsafe { &mut *(ptr as *mut Counter) };

        counter.increment();
        Ok(())
    });

    outcome.resolve::<jni::errors::ThrowRuntimeExAndDefault>()
}

#[jni_mangle("com.chainlynx8248.Lynx_sim.VendorJNI")]
pub unsafe fn counter_value<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    ptr: jlong,
) -> jint {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        // SAFETY: We cant really do anything here we just have to trust that the java passes a valid pointer
        let counter = unsafe { &mut *(ptr as *mut Counter) };
        Ok(counter.count)
    });

    outcome.resolve::<jni::errors::ThrowRuntimeExAndDefault>()
}

#[jni_mangle("com.chainlynx8248.Lynx_sim.VendorJNI")]
pub unsafe fn dealloc_counter<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    ptr: jlong,
) {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        // SAFETY: We cant really do anything here we just have to trust that the java passes a valid pointer
        let _ = unsafe { Box::from_raw(ptr as *mut Counter) };

        Ok(())
    });

    outcome.resolve::<jni::errors::ThrowRuntimeExAndDefault>()
}
