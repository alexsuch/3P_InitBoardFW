### Just before building in VSCode

```
Remove-Item -Recurse -Force .\build\Debug
cmake --preset Debug
cmake --build .\build\Debug
```
