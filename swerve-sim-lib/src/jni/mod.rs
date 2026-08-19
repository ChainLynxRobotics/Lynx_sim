use jni::{
    EnvUnowned,
    errors::Error,
    objects::{JClass, JString},
};

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
