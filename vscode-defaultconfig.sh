#!/bin/sh

if [ -e .vscode/settings.json.default ]
 then cp .vscode/settings.json.default .vscode/settings.json
fi

if [ -e .vscode/tasks.json.default ]
 then cp .vscode/tasks.json.default .vscode/tasks.json
fi

if [ -e .vscode/launch.json.default ]
 then cp .vscode/launch.json.default .vscode/launch.json
fi

if [ -e .vscode/extensions.json.default ]
 then cp .vscode/extensions.json.default .vscode/extensions.json
fi