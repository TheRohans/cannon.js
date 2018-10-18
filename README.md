# 大炮 (Dàpào)

![Build Status](https://codebuild.us-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiVmcwUTAvM24vTWhrZ1NTUDJXL1Y2bWdVOHZtaDQ0ZUt2ZDNBaGp2QVdCUVVJM3BPS0ZDRk1IRVo3RGdvS0lkak1iSGRtZDl5dUJ1R1lxVUV3QTFmOTBnPSIsIml2UGFyYW1ldGVyU3BlYyI6IjJEakF3UDlsbXBMa0ZnRmkiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=master)

## Lightweight 3D Physics for the Web and Beyond

Dapao is a Typescript port of the excellent [cannon.js](https://github.com/schteppe/cannon.js) library.

## Project Goals

I am porting this to Typescript so I can use it within my own game engine (which is in Typescript). While the cannon.js version works perfectly well, I want to have better integration with my engine (tree shaking, etc), and I eventually want to be able to compile this to _webassembly_ or maybe even native - things Javascript can't do at the moment.

## Using the library

If you want to use the library, you can either reference this repo directly from within your webpack config:

    ...
    "dependencies": {
      "dapao": "https://github.com/TheRohans/dapao.git#master",
      "document-register-element": "^1.7.0"
      ...
    },
    ...

Or clone the repo and reference it from the local file system:

    ...
    "dependencies": {
      "dapao": "file:../dapao",
      "document-register-element": "^1.7.0"
      ...
    },
    ...

## Contributing

1. Clone this repository
2. Install [node.js](https://nodejs.org/en/)
3. Run ```npm install``` to install all of the dependencies
4. Run ```npm run build``` to build DaPao


## Example

For deeper examples, I recommend you checkout the cannon.js project ([cannon.js](https://github.com/schteppe/cannon.js)), but here is a very basic example:

The sample code below creates a sphere on a plane, steps the simulation, and prints the sphere simulation to the console. Note that DaPao (and Cannon.js) uses [SI units](http://en.wikipedia.org/wiki/International_System_of_Units) (metre, kilogram, second, etc.).

```typescript
import * as DaPao from 'dapao';

// Setup our world
var world = new DaPao.World();
world.gravity.set(0, 0, -9.82); // m/s²

// Create a sphere
var radius = 1; // m
var sphereBody = new DaPao.Body({
   mass: 5, // kg
   position: new DaPao.Vec3(0, 0, 10), // m
   shape: new DaPao.Sphere(radius)
});
world.addBody(sphereBody);

// Create a plane
var groundBody = new DaPao.Body({
    mass: 0 // mass == 0 makes the body static
});
var groundShape = new DaPao.Plane();
groundBody.addShape(groundShape);
world.addBody(groundBody);

var fixedTimeStep = 1.0 / 60.0; // seconds
var maxSubSteps = 3;

// Start the simulation loop
var lastTime;
(function simloop(time){
  requestAnimationFrame(simloop);
  if(lastTime !== undefined){
     var dt = (time - lastTime) / 1000;
     world.step(fixedTimeStep, dt, maxSubSteps);
  }
  console.log("Sphere z position: " + sphereBody.position.z);
  lastTime = time;
})();
```

## Features

* Rigid body dynamics
* Discrete collision detection
* Contacts, friction and restitution
* Collision filters
* Body sleeping
* Various shapes and collision algorithms (see table below)

|             | Sphere | Plane | Box | Convex | Particle | Heightfield | Trimesh |
| :-----------|:------:|:-----:|:---:|:------:|:--------:|:-----------:|:-------:|
| Sphere      | Yes    | Yes   | Yes | Yes    | Yes      | (todo)      | (todo)  |
| Plane       | -      | -     | Yes | Yes    | Yes      | -           | (todo)  |
| Box         | -      | -     | Yes | Yes    | Yes      | (todo)      | (todo)  |
| Cylinder    | -      | -     | Yes | Yes    | Yes      | (todo)      | (todo)  |
| Convex      | -      | -     | -   | Yes    | Yes      | (todo)      | (todo)  |
| Particle    | -      | -     | -   | -      | -        | (todo)      | (todo)  |
| Heightfield | -      | -     | -   | -      | -        | -           | (todo)  |
| Trimesh     | -      | -     | -   | -      | -        | -           | -       |
