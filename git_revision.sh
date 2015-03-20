#!/bin/sh

echo "// AUTOMATICALLY GENERATED" > src/git_revision.c
echo "const char * build_git_version = \"$(git describe --dirty --always --tags)\";" >> src/git_revision.c
echo "const char * build_git_sha = \"$(git rev-parse HEAD)\";" >> src/git_revision.c
echo "const char * build_git_branch = \"$(git rev-parse --abbrev-ref HEAD)\";" >> src/git_revision.c
echo "const char * build_date = \"$(date -u +"%Y-%m-%dT%H:%M:%SZ")\";" >> src/git_revision.c
