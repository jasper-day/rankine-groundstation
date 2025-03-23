<script lang="ts">
    import "cesium/Build/Cesium/Widgets/widgets.css";

    import { Cartesian3, Ion, Math as CesiumMath, Terrain, Viewer } from 'cesium';
    import { Arc, Line, Local3, ORIGIN } from "$lib/geometry";
    import "cesium/Build/Cesium/Widgets/widgets.css";
    import { onMount } from 'svelte';

    Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

    const lines: Line[] = [];//new Line(new Local3(0, 0, 0), new Local3(0, 20, 0))];
    const arcs: Arc[] = [new Arc(new Local3(0, 0, 0), 20, Math.PI/2, Math.PI * 3 / 2)];

    onMount(() => {
        // Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
        const viewer = new Viewer('cesiumContainer', {
          terrain: Terrain.fromWorldTerrain(),
        });    

        // Fly the camera to the origin longitude, latitude, and height.
        viewer.camera.flyTo({
          destination: Cartesian3.fromRadians(ORIGIN.longitude, ORIGIN.latitude, 1000),
          orientation: {
            heading: CesiumMath.toRadians(0.0),
            pitch: CesiumMath.toRadians(-90.0),
          },
          duration: 0,
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

</script>

<div id="cesiumContainer" style="height:max-content;"></div>
