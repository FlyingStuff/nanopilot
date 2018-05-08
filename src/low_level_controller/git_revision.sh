#!/bin/sh

echo "// AUTOMATICALLY GENERATED" > src/git_revision.c
echo "const char * build_git_sha = \"$(git rev-parse HEAD)\";" >> src/git_revision.c
