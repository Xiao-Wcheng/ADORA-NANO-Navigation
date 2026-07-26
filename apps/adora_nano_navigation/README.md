# Navigation application entry points

This directory contains the production Dora YAML and Python orchestration for mapping, localization, and autonomous navigation. Start with the project-level `README.md` and `docs/OPERATIONS.md`.

Primary commands:

```bash
python3 apps/adora_nano_navigation/adora_nav.py map --clean
python3 apps/adora_nano_navigation/adora_nav.py nav --localize --relative 0.30 -0.15 --global-relative
python3 apps/adora_nano_navigation/adora_nav.py stop
```

Generated YAML and Dora session/output files are runtime artifacts and are not part of the source distribution.
