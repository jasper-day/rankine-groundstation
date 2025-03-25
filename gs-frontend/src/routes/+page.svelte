<script lang="ts">
	import 'cesium/Build/Cesium/Widgets/widgets.css';

	import {
		Cartesian3,
		Ion,
		Math as CesiumMath,
		Terrain,
		Viewer,
		Cartesian2
	} from 'cesium';
	import { Arc, HANDLE_POINT_RADIUS, Line, Local3, ORIGIN } from '$lib/geometry';
	import 'cesium/Build/Cesium/Widgets/widgets.css';
	import { onMount } from 'svelte';

	Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

	const shapes: (Arc | Line)[] = [];// [new Arc(new Local3(0, 0, 0), 20, Math.PI / 2, (Math.PI * 3) / 2)];

	let viewer: Viewer | undefined;
	let ctx: CanvasRenderingContext2D | null = null;
	let tool: 'Line' | 'Arc' | 'Empty' = 'Empty';
	let intermediate_points: Local3[] = [];
	let arc_direction_guess: 1 | -1 | undefined;
	let prelim_arc_direction_guess: 1 | -1 | undefined; // this is for rendering the arc preview when we are not really confident in the direction guess yet
	// to avoid instantiating objects continuously
	// may be premature optimisation but cesium does it so i will too
	let scratchc3_a: Cartesian3 = new Cartesian3();
	let scratchc3_b: Cartesian3 = new Cartesian3();

	onMount(() => {
		// Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
		viewer = new Viewer('cesiumContainer', {
			terrain: Terrain.fromWorldTerrain()
		});
		ctx = canvas.getContext('2d');
		canvas.width = viewer.canvas.width;
		canvas.height = viewer.canvas.height;

		// Fly the camera to the origin longitude, latitude, and height.
		viewer.camera.flyTo({
			destination: Cartesian3.fromRadians(ORIGIN.longitude, ORIGIN.latitude, 1000),
			orientation: {
				heading: CesiumMath.toRadians(0.0),
				pitch: CesiumMath.toRadians(-90.0)
			},
			duration: 0
		});
		viewer.camera.switchToOrthographicFrustum();

		viewer.clock.onTick.addEventListener((_: any) => {
			if (viewer && ctx) {
				ctx.clearRect(0, 0, canvas.width, canvas.height);
				draw_tooltip(mouseX, mouseY);
				ctx.strokeStyle = 'yellow';
				ctx.fillStyle = "#ffd040";
				for (const s of shapes) {
					s.draw(ctx, viewer);
				}


				if (intermediate_points.length > 0) {
					// some shape is being defined!! draw it
					// TODO costly operation to undo later on...
					const mouse_cartesian = viewer.camera.pickEllipsoid(
						new Cartesian3(mouseX, mouseY),
						viewer.scene.ellipsoid
					);
					if (!mouse_cartesian) return;
					const mouse_local = Local3.fromCartesian(mouse_cartesian);

					if (tool == "Line") {
						new Line(intermediate_points[0], mouse_local).draw(ctx, viewer);
					}
					if (tool == "Arc" && intermediate_points.length > 0) {
						if (intermediate_points.length == 1) {
							let centre = intermediate_points[0];
							let outer_point = mouse_local;
							let r = Local3.distance(centre, outer_point);
							new Arc(centre, r, 0, Math.PI * 2, 1).draw(ctx, viewer, true);
						} else if (intermediate_points.length == 2) {
							let dir: 1 | -1;
							if (arc_direction_guess === undefined) {
								if (prelim_arc_direction_guess !== undefined) {
									dir = prelim_arc_direction_guess;
								} else {
									dir = 1;
								}
							} else {
								dir = arc_direction_guess;
							}
							Arc.from_centre_and_points(intermediate_points[0], intermediate_points[1], mouse_local, dir).draw(ctx, viewer);
						}
					}
				}
			}
		});

		// Add Cesium OSM Buildings, a global 3D buildings layer.
		// createOsmBuildingsAsync().then((buildingTileset) => viewer.scene.primitives.add(buildingTileset));
		
		// Chrome doesn't support mouse events
		viewer.cesiumWidget.canvas.addEventListener("pointerdown", mousedown);
		viewer.cesiumWidget.canvas.addEventListener("pointerup", mouseup);
	});

	let drag_object: { shape_index: number, point_index: number } | undefined = undefined;
	function mousedown(event: MouseEvent) {
		// console.log(event);
		if (viewer !== undefined) {
			const cartesian = viewer.camera.pickEllipsoid(
				new Cartesian3(event.clientX, event.clientY),
				viewer.scene.ellipsoid
			);
			if (!cartesian) return; // just ignore an invalid position

			if (tool == "Line") {
				if (intermediate_points.length == 0) {
					// add first point
					intermediate_points.push(Local3.fromCartesian(cartesian));
				} else {
					shapes.push(new Line(intermediate_points[0], Local3.fromCartesian(cartesian)));
					intermediate_points = [];
				}
			} else if (tool == "Arc") {
				if (intermediate_points.length < 2) {
					intermediate_points.push(Local3.fromCartesian(cartesian));
				} else {
					// we really should have a guess for the direction of the arc by now. Try to use a preliminary guess if we have one
					if (arc_direction_guess === undefined) {
						if (prelim_arc_direction_guess !== undefined) {
							arc_direction_guess = prelim_arc_direction_guess;
						} else {
							arc_direction_guess = 1;
						}
					}
					shapes.push(Arc.from_centre_and_points(intermediate_points[0], intermediate_points[1], Local3.fromCartesian(cartesian), arc_direction_guess));
					arc_direction_guess = undefined;
					prelim_arc_direction_guess = undefined;
					intermediate_points = [];
				}
			} else if (tool == "Empty") {
				// check for dragging
				const mouse = new Cartesian2(event.clientX, event.clientY);
				let idx = 0;
				const max_sq_dist = HANDLE_POINT_RADIUS * HANDLE_POINT_RADIUS;
				for (const s of shapes) {
					if (s instanceof Line) {
						const a = viewer.scene.cartesianToCanvasCoordinates(s.start.toCartesian(), scratchc3_a);
						const b = viewer.scene.cartesianToCanvasCoordinates(s.end.toCartesian(), scratchc3_b);
						
						if (Cartesian2.distanceSquared(a, mouse) < max_sq_dist) {
							drag_object = { shape_index: idx, point_index: 0 };
						}
						if (Cartesian2.distanceSquared(b, mouse) < max_sq_dist) {
							drag_object = { shape_index: idx, point_index: 1 };
						}
					}
					idx += 1;
				}
				if (drag_object !== undefined) {
					viewer.scene.screenSpaceCameraController.enableInputs = false;
				}
			}
		}
	}

	function mouseup(_: MouseEvent) {
		if (drag_object !== undefined) {
			if (viewer) viewer.scene.screenSpaceCameraController.enableInputs = true;
			drag_object = undefined;
		}
	}
	
	function keypress(event: KeyboardEvent) {
		if (event.key == 'l') {
			tool = 'Line';
		} else if (event.key == 'a') {
			tool = 'Arc';
		} else if (event.key == "Escape") {
			tool = 'Empty';
			intermediate_points = [];
			arc_direction_guess = undefined;
			prelim_arc_direction_guess = undefined;
		}
		draw_tooltip(mouseX, mouseY);
	}

	function abs_angle(a: number): number {
	 	const pi2 = Math.PI * 2;
		return (pi2 + (a % pi2)) % pi2;
	}

	let cleared = false;
	let mouseX: number, mouseY: number;
	function mousemove(event: MouseEvent) {
		mouseX = event.clientX;
		mouseY = event.clientY;
		if (drag_object !== undefined && viewer !== undefined) {
			const ellipsoid = viewer.scene.ellipsoid;
			const cartesian = viewer.camera.pickEllipsoid(
				new Cartesian3(event.clientX, event.clientY),
				ellipsoid
			);
			if (!cartesian) return; // just ignore an invalid position
			let local = Local3.fromCartesian(cartesian);
			let s = shapes[drag_object.shape_index];
			if (s instanceof Line) {
				if (drag_object.point_index == 0) {
					s.start = local;
				}
				if (drag_object.point_index == 1) {
					s.end = local;
				}
			} else if (s instanceof Arc) {
				
			}
		}
		// we are almost done making the arc, defining the last point, but we haven't figured out what direction it's going in yet (TODO inaccurate comment maybe)
		if (tool == "Arc" && intermediate_points.length == 2 && viewer !== undefined) {
			// guess
			const centre = intermediate_points[0];
			const a = intermediate_points[1];
			const cartesian = viewer.camera.pickEllipsoid(
				new Cartesian3(event.clientX, event.clientY),
				viewer.scene.ellipsoid
			);
			// ignore invalid position...
			if (cartesian === undefined) return;
			const b = Local3.fromCartesian(cartesian);
	        const theta0 = -Math.atan2(a.y - centre.y, a.x - centre.x);
	        const theta1 = -Math.atan2(b.y - centre.y, b.x - centre.x);

	        // this is awful and slow
	        // why is it so hard to find the signed difference between two angles??
	        let delta = Math.atan2(Math.sin(theta0 - theta1), Math.cos(theta0 - theta1));
	        prelim_arc_direction_guess = delta > 0 ? -1 : 1;
	        // don't guess for very tiny (possibly zero) difference
	        // in fact, reset so we can guess again
	        if (Math.abs(delta) < 0.15) {
	        	arc_direction_guess = undefined;
	        	return;
	        }

	        // do not overwrite an existing guess
			if (arc_direction_guess === undefined) {
		        arc_direction_guess = prelim_arc_direction_guess;
	        }
		}
	}

	function draw_tooltip(x: number, y: number) {
		if (tool === 'Empty' && cleared) {
			return;
		}
		if (ctx) {
			// ctx.clearRect(0, 0, canvas.width, canvas.height);
			if (tool === 'Empty' && !cleared) {
				cleared = true;
				return;
			}
			cleared = false;
			x -= 16;
			y += 16;
			ctx.strokeStyle = 'red';
			if (tool == 'Arc') {
				ctx.beginPath();
				ctx.arc(x, y, 10, Math.PI / 2, (2 * Math.PI) / 3 + Math.PI / 2);
			} else if (tool == 'Line') {
				ctx.beginPath();
				ctx.moveTo(x, y);
				ctx.lineTo(x + 16, y + 16);
			}
			ctx.stroke();
		}
	}
	let canvas: HTMLCanvasElement;
</script>

<canvas id="canvas" style="z-index: 2; position:absolute; pointer-events: none;" bind:this={canvas}
></canvas>
<div
	id="cesiumContainer"
	style="height:max-content; z-index: 1;position:relative;"
></div>
<svelte:window on:keydown={keypress} on:mousemove|preventDefault={mousemove} />
