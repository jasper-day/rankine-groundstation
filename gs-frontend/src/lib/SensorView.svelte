<script lang="ts">
    export let sensors: [string, number[], string][];
    let graphs: (HTMLCanvasElement | undefined)[] = [];
    let colours: string[] = [];
    function HSVtoRGB(h: number, s: number, v: number) {
        let r, g, b;
        let i = Math.floor(h * 6);
        let f = h * 6 - i;
        let p = v * (1 - s);
        let q = v * (1 - f * s);
        let t = v * (1 - (1 - f) * s);
        switch (i % 6) {
            case 0:
                (r = v), (g = t), (b = p);
                break;
            case 1:
                (r = q), (g = v), (b = p);
                break;
            case 2:
                (r = p), (g = v), (b = t);
                break;
            case 3:
                (r = p), (g = q), (b = v);
                break;
            case 4:
                (r = t), (g = p), (b = v);
                break;
            case 5:
                (r = v), (g = p), (b = q);
                break;
            default:
                r = 0;
                g = 0;
                b = 0;
        }
        const pad = (x: string) => "0".repeat(2 - x.length) + x;
        return (
            "#" +
            pad(Math.round(r * 255).toString(16)) +
            pad(Math.round(g * 255).toString(16)) +
            pad(Math.round(b * 255).toString(16))
        );
    }
    export function draw_graphs() {
        for (let i = 0; i < sensors.length; i++) {
            const canvas = graphs[i];
            if (!canvas) continue;
            let ctx = canvas.getContext("2d");
            if (!ctx) continue;
            let colour = colours[i];
            if (!colour) colours[i] = HSVtoRGB(i / sensors.length, 0.8, 0.8);
            canvas.width = canvas.clientWidth;
            canvas.height = canvas.clientHeight;
            const n_points = 100;
            const point_spacing = canvas.width / (n_points - 2);
            let x = point_spacing;
            const begin_idx = Math.max(0, sensors[i][1].length - n_points);
            let min = Math.min(...sensors[i][1].slice(begin_idx));
            let max = Math.max(...sensors[i][1].slice(begin_idx));
            if (max - min < 0.0001) {
                min -= 0.01;
                max += 0.01;
            }
            let scl = (max - min) / canvas.height;
            ctx.lineWidth = 2;
            for (let j = begin_idx; j < sensors[i][1].length - 2; j++) {
                ctx.strokeStyle = colour;
                ctx.fillStyle = colour;
                ctx.beginPath();
                ctx.moveTo(x - point_spacing, canvas.height - (sensors[i][1][j] - min) / scl);
                ctx.lineTo(x, canvas.height - (sensors[i][1][j + 1] - min) / scl);
                ctx.stroke();
                // ctx.beginPath();
                // ctx.arc(x, canvas.height - (sensors[i][1][j + 1] - min) / scl, 2, 0, Math.PI * 2);
                // ctx.fill();
                x += point_spacing;
            }
            ctx.lineWidth = 1;
        }
    }
</script>

<div id="grid">
    {#each sensors as sensor, n}
        <p style="grid-row: {n + 1}" class="label">{sensor[0]}</p>
        <canvas style="grid-row: {n + 1}" class="graph" bind:this={graphs[n]}></canvas>
        <p style="grid-row: {n + 1}" class="value">{sensor[1][sensor[1].length - 1].toFixed(2)} {sensor[2]}</p>
    {/each}
</div>

<style>
    p {
        background-color: #222222;
        color: #dddddd;
        white-space: nowrap;
    }

    #grid {
        background-color: #222222;
        display: grid;
        grid-template-columns: min-content 2fr min-content;
        gap: 1px;
        height: 100%;
    }

    .label {
        width: fit-content;
        grid-column: 1;
        margin-top: auto;
        margin-bottom: auto;
    }

    .value {
        width: fit-content;
        grid-column: 3;
        margin-top: auto;
        margin-bottom: auto;
    }

    .graph {
        width: 100%;
        background-color: #181818;
        grid-column: 2;
        height: 44px;
    }
</style>
