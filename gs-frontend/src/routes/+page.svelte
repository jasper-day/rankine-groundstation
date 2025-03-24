<script lang="ts">
	import 'cesium/Build/Cesium/Widgets/widgets.css';

	import {
		Cartesian3,
		Ion,
		Math as CesiumMath,
		Terrain,
		Viewer,
		Entity,
		ScreenSpaceEventHandler,
		ScreenSpaceEventType,
		defined,

		Matrix4

	} from 'cesium';
	import { Arc, Line, Local3, ORIGIN } from '$lib/geometry';
	import 'cesium/Build/Cesium/Widgets/widgets.css';
	import { onMount } from 'svelte';

	Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

	const lines: Line[] = []; //new Line(new Local3(0, 0, 0), new Local3(0, 20, 0))];
	const arcs: Arc[] = [new Arc(new Local3(0, 0, 0), 20, Math.PI / 2, (Math.PI * 3) / 2)];

	let viewer: Viewer | undefined;
	let tool: 'Line' | 'Arc' | 'Empty' = 'Empty';

	onMount(() => {
		// Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
		viewer = new Viewer('cesiumContainer', {
			terrain: Terrain.fromWorldTerrain()
		});
		canvas.width = viewer.canvas.width;
		canvas.height = viewer.canvas.height;

		// viewer.canvas.setAttribute("tabindex", "0"); // apparently needed for getting keyboard events
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

		// Add Cesium OSM Buildings, a global 3D buildings layer.
		// createOsmBuildingsAsync().then((buildingTileset) => viewer.scene.primitives.add(buildingTileset));

		for (const l of lines) {
			viewer.entities.add(l.toEntity());
		}
		for (const a of arcs) {
			let entities = a.toEntities();
			for (const e of entities) {
				viewer.entities.add(e);
			}
		}
	});

	function map_clicked(event: MouseEvent) {
		// console.log(event);
		if (viewer !== undefined) {
			// const ellipsoid = viewer.scene.ellipsoid;
			// const cartesian = viewer.camera.pickEllipsoid(
			// 	new Cartesian3(event.clientX, event.clientY),
			// 	ellipsoid
			// );
			// if (cartesian !== undefined) {
			// 	line = viewer.entities.add(
			// 		new Line(new Local3(0, 0, 0), Local3.fromCartesian(cartesian)).toEntity()
			// 	);
			// }
		}
	}
	function keypress(event: KeyboardEvent) {
		// console.log(event.keyCode);
		if (event.key == 'l') {
			tool = 'Line';
		} else if (event.key == 'a') {
			tool = 'Arc';
		} else if (event.keyCode == 27) {
			tool = 'Empty';
		}
		draw_tooltip(mouseX, mouseY);
	}
	let cleared = false;
	let mouseX: number, mouseY: number;
	function mousemove(event: MouseEvent) {
		mouseX = event.clientX;
		mouseY = event.clientY;
		draw_tooltip(mouseX, mouseY);
		if (viewer) {
			let ctx = canvas.getContext('2d');
			let a = viewer.camera.viewMatrix;
			let b = viewer.camera.frustum.projectionMatrix;
			let v = new Cartesian3();
			Matrix4.multiplyByPoint(a, new Local3(0, 0, 0).toCartesian(), v);
			viewer.camera.frustum.computeCullingVolume
			console.log("VEctor:", v);
			if (ctx) {
				ctx.clearRect(0, 0, canvas.width, canvas.height);
				ctx.strokeStyle = 'yellow';
				ctx.beginPath();
				ctx.arc(v.x, -v.y, 10, 0, 2 * Math.PI);
				ctx.stroke();
			}
			
		}
		// if (line && viewer) {
		// 	const ellipsoid = viewer.scene.ellipsoid;
		// 	const cartesian = viewer.camera.pickEllipsoid(
		// 		new Cartesian3(event.clientX, event.clientY),
		// 		ellipsoid
		// 	);
		// 	// line.corridor.positions = [new Local3(0, 0, 0).toCartesian(), cartesian];
		// }
	}

	function draw_tooltip(x: number, y: number) {
		// if (tool === 'Empty' && cleared) {
		// 	return;
		// }
		// let ctx = canvas.getContext('2d');
		// if (ctx) {
		// 	ctx.clearRect(0, 0, canvas.width, canvas.height);
		// 	if (tool === 'Empty' && !cleared) {
		// 		cleared = true;
		// 		return;
		// 	}
		// 	cleared = false;
		// 	x -= 16;
		// 	y += 16;
		// 	ctx.strokeStyle = 'red';
		// 	if (tool == 'Arc') {
		// 		ctx.beginPath();
		// 		ctx.arc(x, y, 10, Math.PI / 2, (2 * Math.PI) / 3 + Math.PI / 2);
		// 	} else if (tool == 'Line') {
		// 		ctx.beginPath();
		// 		ctx.moveTo(x, y);
		// 		ctx.lineTo(x + 16, y + 16);
		// 	}
		// 	ctx.stroke();
		// }
	}
	let canvas: HTMLCanvasElement;
</script>

<canvas id="canvas" style="z-index: 2; position:absolute; pointer-events: none;" bind:this={canvas}
></canvas>
<div
	id="cesiumContainer"
	style="height:max-content; z-index: 1;position:relative;"
	on:click={map_clicked}
></div>
<svelte:window on:keydown={keypress} on:mousemove={mousemove} />
