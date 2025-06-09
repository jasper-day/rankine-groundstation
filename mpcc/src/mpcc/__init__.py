import jax

# I really, really hate floats
jax.config.update("jax_enable_x64", True)
