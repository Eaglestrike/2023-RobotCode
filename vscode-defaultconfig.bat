@echo off

IF EXIST ".vscode\settings.json.default" (COPY /Y ".vscode\settings.json.default" ".vscode\settings.json")
IF EXIST ".vscode\tasks.json.default" (COPY /Y ".vscode\tasks.json.default" ".vscode\tasks.json")
IF EXIST ".vscode\launch.json.default" (COPY /Y ".vscode\launch.json.default" ".vscode\launch.json")
IF EXIST ".vscode\extensions.json.default" (COPY /Y ".vscode\extensions.json.default" ".vscode\extensions.json")