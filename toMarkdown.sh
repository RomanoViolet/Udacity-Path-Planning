#!/usr/bin/env bash

# Requires pandoc v2.2.1 or greater.

# Use the pandoc utility to convert the document from .odt format to markdown (.md) format appropriate for Github
# pandoc -s --from=odt --to=markdown_github --output=ReadMe.md ReadMe.odt

# Use the pandoc utility to convert the document from LaTex (.tex) format to markdown (.md) format appropriate for Github
pandoc -s ReadMe.yaml --standalone --indented-code-classes=cpp --highlight-style=breezedark --listings --from=latex --to=gfm --output=ReadMe.md ReadMe.tex
