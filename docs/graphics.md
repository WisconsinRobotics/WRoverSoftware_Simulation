# Graphics information
WSL is supper funky when it comes to graphics. You might need to make some changes in order to get this to render on your computer


## INTEL Arc Graphics cards
Intel Arc graphics cards have proven problematic, you need to use software rendering if
possible by your computer.

```bash
# Run the following in your shell instance
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
```
