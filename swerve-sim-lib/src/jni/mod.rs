use jni::{
    EnvUnowned,
    errors::Error,
    objects::{JClass, JString},
    sys::{jint, jlong},
};

#[derive(Default)]
struct Counter {
    count: i32,
}
impl Counter {
    fn increment(&mut self) {
        self.count += 1;
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_Joe_hello<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    class: JClass<'caller>,
    input: JString<'caller>,
) -> JString<'caller> {
    let outcome = unowned_env.with_env(|env| -> Result<_, Error> {
        let input = input.to_string();
        let class_name = class.get_name(env)?.to_string();
        JString::from_str(env, format!("Hello, {}. \nFrom {}.", input, class_name))
    });

    outcome.resolve::<jni::errors::ThrowRuntimeExAndDefault>()
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_Joe_newCounter<'caller>(
    mut unowned_env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
) -> jlong {
    let outcome = unowned_env.with_env(|_env| -> Result<_, Error> {
        Ok(Box::into_raw(Box::new(Counter::default())) as jlong)
    });

    outcome.resolve::<jni::errors::ThrowRuntimeExAndDefault>()
}

#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_Joe_incrementCounter<'caller>(
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

#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_Joe_counterValue<'caller>(
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

#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_Joe_deallocCounter<'caller>(
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
