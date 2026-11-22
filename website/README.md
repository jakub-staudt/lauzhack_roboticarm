# Reacha website

Static single-page site for Reacha (gaze-controlled assistive robotics): hero, an in-browser 3D simulation of the SO-101 arm
("live lab"), how it works, system architecture, safety and contact. No build step.

- `index.html`: the whole page (HTML, CSS and JS). Loads Inter from Google Fonts and three.js r128 from cdnjs.
- `assets/`: the hero image, logo and the SO-101 meshes (base64 `.txt`, decoded in the page).
- `.nojekyll`: tells GitHub Pages to serve the files as they are.

## Preview locally

```sh
cd website && python3 -m http.server 8000   # http://localhost:8000
```

## GitHub Pages

Settings > Pages > Deploy from a branch > choose the branch and the `/website` folder is not offered directly
(Pages only allows `/` or `/docs`), so either move/copy `website/` to `docs/`, or publish it with a GitHub Actions
workflow that uploads `website/` as the Pages artifact.
