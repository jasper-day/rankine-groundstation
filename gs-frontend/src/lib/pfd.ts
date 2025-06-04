
import wings_src from "$lib/assets/wings.png";
const wings = new Image();
wings.src = wings_src;

const pitchbar_height = 500;
const roll_line_len = 15;
const arrow_clearance = 10;
const roll_range = 40;

type Ctx = CanvasRenderingContext2D;

function draw_background(ctx: Ctx, w: number, h: number, pitch: number, roll: number, max_length: number) {
    const pitch_factor = pitchbar_height * pitch;
    const dx0 = max_length * Math.cos(Math.PI + roll);
    const dy0 = max_length * Math.sin(Math.PI + roll);
    const dx1 = max_length * Math.cos(roll)
    const dy1 = max_length * Math.sin(roll)
    const x0 = dx0 + w/2 + pitch_factor * Math.sin(-roll);
    const y0 = dy0 + h/2 + pitch_factor * Math.cos(roll);
    const x1 = dx1 + w/2 + pitch_factor * Math.sin(-roll);
    const y1 = dy1 + h/2 + pitch_factor * Math.cos(roll);

    const x3 = x0 - 2 * dy0;
    const y3 = y0 + 2 * dx0;
    const x2 = x1 - 2 * dy0;
    const y2 = y1 + 2 * dx0;
    const x5 = x0 + 2 * dy0;
    const y5 = y0 - 2 * dx0;
    const x4 = x1 + 2 * dy0;
    const y4 = y1 - 2 * dx0;

    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#000000";
    ctx.fillRect(0, 0, w, h);

    ctx.fillStyle = "#5b93c5";
    ctx.beginPath()
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.lineTo(x3, y3);
    ctx.lineTo(x0, y0);
    ctx.fill();

    ctx.fillStyle = "#7d5233";
    ctx.beginPath()
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.lineTo(x4, y4);
    ctx.lineTo(x5, y5);
    ctx.lineTo(x0, y0);
    ctx.fill();
}

function draw_pitch_bars(ctx: Ctx, w: number, h: number, pitch: number, roll: number, max_length: number) {
    ctx.strokeStyle = "#ffffff";
    ctx.fillStyle = "#dddddd";
    ctx.font = "20px sans-serif";
    ctx.textBaseline = "middle";
    for (let theta = -90; theta < 90; theta += 2.5) {
        let bar_len = 20;
        if (Math.abs(theta) < 0.01) {
            bar_len = max_length * 2;
        } else if (Math.abs(theta % 10) < 0.01) {
            bar_len = 80;
        } else if (Math.abs(theta % 5) < 0.01) {
            bar_len = 40;
        }
        const pitch_adjust = (pitch + theta / 180 * Math.PI) * pitchbar_height;
        const x0 = bar_len * Math.cos(Math.PI + roll) + w/2 + pitch_adjust * Math.sin(-roll);
        const y0 = bar_len * Math.sin(Math.PI + roll) + h/2 + pitch_adjust * Math.cos(roll);
        const x1 = bar_len * Math.cos(roll)           + w/2 + pitch_adjust * Math.sin(-roll);
        const y1 = bar_len * Math.sin(roll)           + h/2 + pitch_adjust * Math.cos(roll);
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
        if (bar_len == 80) {
            ctx.save();
            ctx.translate(x0, y0);
            ctx.rotate(roll);
            ctx.fillText(Math.abs(theta).toString(), -30, 0);
            ctx.fillText(Math.abs(theta).toString(), bar_len * 2 + 5, 0);
            ctx.restore();
        }
    }
}

function draw_roll_indicator_background(ctx: Ctx, w: number, h: number, x0: number, y0: number, outer_radius: number) {
    ctx.fillStyle = "#5b93c5";
    ctx.beginPath();
    const radius = outer_radius - roll_line_len - arrow_clearance;
    const x = radius * Math.cos(-roll_range / 180 * Math.PI -Math.PI/2) + x0;
    const y = radius * Math.sin(-roll_range / 180 * Math.PI -Math.PI/2) + y0;
    ctx.moveTo(x, y);
    for (let theta = -roll_range; theta <= roll_range; theta += 10) {
        const theta_rad = theta / 180 * Math.PI - Math.PI/2;
        const x = radius * Math.cos(theta_rad) + x0;
        const y = radius * Math.sin(theta_rad) + y0;
        ctx.lineTo(x, y);
    }
    ctx.lineTo(w, y);
    ctx.lineTo(w, 0);
    ctx.lineTo(0, 0);
    ctx.lineTo(0, y);
    ctx.fill();
}

function draw_roll_bars(ctx: Ctx, centre_x: number, centre_y: number, outer_radius: number) {
    ctx.fillStyle = "#dddddd";

    for (let theta = -roll_range; theta <= roll_range; theta += 10) {
        const theta_rad = theta / 180 * Math.PI - Math.PI/2;
        const x0 = outer_radius * Math.cos(theta_rad) + centre_x;
        const y0 = outer_radius * Math.sin(theta_rad) + centre_y;
        const x1 = (outer_radius - roll_line_len) * Math.cos(theta_rad) + centre_x;
        const y1 = (outer_radius - roll_line_len) * Math.sin(theta_rad) + centre_y;
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();

        ctx.save();
        ctx.translate(x0, y0);
        ctx.rotate(theta_rad + Math.PI/2);
        ctx.fillText(Math.abs(theta).toString(), -ctx.measureText(Math.abs(theta).toString()).width / 2, -10);
        ctx.restore();
    }
}

function draw_roll_pointer(ctx: Ctx, w: number, h: number, roll: number, outer_radius: number) {
    ctx.lineWidth = 2;
    const r0 = outer_radius - roll_line_len - arrow_clearance - 2;
    const x0 = (r0) * Math.cos(roll + 0.06 - Math.PI/2) + w/2;
    const y0 = (r0) * Math.sin(roll + 0.06 - Math.PI/2) + h/2;
    const x1 = (r0) * Math.cos(roll - 0.06 - Math.PI/2) + w/2;
    const y1 = (r0) * Math.sin(roll - 0.06 - Math.PI/2) + h/2;
    const x2 = (r0 + 10) * Math.cos(roll  - Math.PI/2) + w/2;
    const y2 = (r0 + 10) * Math.sin(roll  - Math.PI/2) + h/2;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.lineTo(x0, y0);
    ctx.stroke();
}

function draw_roll_indicator(ctx: Ctx, w: number, h: number, roll: number) {
    const centre_x = w/2;
    const centre_y = h/2;
    const roll_r = h/2 - 20;

    draw_roll_indicator_background(ctx, w, h, centre_x, centre_y, roll_r);
    draw_roll_bars(ctx, centre_x, centre_y, roll_r);        
    draw_roll_pointer(ctx, w, h, roll, roll_r);
}

function draw_wings(ctx: Ctx, w: number, h: number) {
    const desired_width = w/2.5;
    const ar = wings.height / wings.width;
    const height = ar * desired_width;
    const centre_x = 237 * desired_width / wings.width, centre_y = 10 * height / wings.height;
    ctx.drawImage(wings, w/2 - centre_x, h/2 - centre_y, desired_width, height);
}

function to_nearest(x: number, n: number): number {
    if (x == 0) return 0;
    return Math.floor(x / n) * n;
}

function draw_tape(ctx: Ctx, w: number, h: number, value: number, inverse: boolean, tape_range: number, tape_step: number) {
    ctx.textAlign = inverse ? "left" : "right";
    ctx.font = "15px sans-serif";
    const tape_height = h * 0.5;
    const tape_start = (h - tape_height) / 2;
    const border_space = 40;
    const lines_start = border_space + ctx.measureText("20").width;
    const tape_left = inverse ? w - lines_start : lines_start;
    const min_tape = Math.max(0, value - tape_range/2);
    const min_tape_boundary = value - tape_range/2;
    const max_tape = value + tape_range/2;
    const rf = inverse ? -1 : 1;
    const line_len =  10;
    const line_space = 5;
    ctx.fillStyle = "#00000020";
    let x = tape_left + rf * (line_space + line_len);
    ctx.fillRect(x, tape_start - line_space, -rf * (30 + line_len + line_space), tape_height + line_space * 2);
    ctx.lineWidth = 1;
    ctx.fillStyle = "#ddddddff";
    for (let s = to_nearest(max_tape, tape_step); s >= min_tape; s -= tape_step) {
        const pos = Math.round(tape_height * (1 - (s - min_tape_boundary) / (max_tape - min_tape_boundary)) + tape_start);
        ctx.beginPath();
        ctx.moveTo(tape_left + rf * line_space, pos);
        ctx.lineTo(tape_left + rf * (line_space + line_len), pos);
        ctx.stroke();
        ctx.fillText(s.toString(), tape_left, pos);
    }

    const boxw = 50;
    const boxh = 30;
    const box_left = inverse ? w - 4 - boxw : 4;
    ctx.font = "18px sans-serif";
    ctx.fillStyle = "#000000ff";
    ctx.strokeStyle = "#ffffffff";
    ctx.textAlign = "right";
    ctx.fillRect(box_left, tape_height / 2 + tape_start - boxh/2, boxw, boxh);
    ctx.strokeRect(box_left, tape_height / 2 + tape_start - boxh/2, boxw, boxh);
    ctx.beginPath();
    const x0 = box_left + (inverse ? 0 : boxw);
    const y0 = tape_height / 2 + tape_start - boxh / 2;
    ctx.moveTo(x0 - 1 * rf, y0 + boxh / 3);
    ctx.lineTo(x0 + 8 * rf, y0 + boxh / 2);
    ctx.lineTo(x0 - 1 * rf, y0 + 2 * boxh / 3);
    ctx.lineTo(x0 - 1 * rf, y0 + boxh / 3);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(x0,     y0 + boxh / 3);
    ctx.lineTo(x0 + 8 * rf, y0 + boxh / 2);
    ctx.lineTo(x0,     y0 + 2 * boxh / 3);
    ctx.stroke();
    ctx.fillStyle = "#ffffff";
    ctx.fillText(value.toFixed(tape_step < 4 ? 2 : 1), box_left + boxw - 2, tape_height / 2 + tape_start);
    ctx.textAlign = "left";
}

export function draw_horizon(pfd: HTMLCanvasElement, ctx: CanvasRenderingContext2D, pitch: number, roll: number, ias: number, alt: number) {
    pfd.width = pfd.clientWidth;
    pfd.height = pfd.clientHeight;
    // let pitch = 0 / 180 * Math.PI;
    // let roll = 0 / 180 * Math.PI;
    // let ias = 2.5;
    // let alt = 130;

    const w = pfd.width;
    const h = pfd.height + 50; // hack to show more sky for the top indicator
    const max_length = Math.max(w, h) + Math.PI * pitchbar_height;

    draw_background(ctx, w, h, pitch, roll, max_length);
    draw_pitch_bars(ctx, w, h, pitch, roll, max_length);
    draw_roll_indicator(ctx, w, h, roll);
    draw_wings(ctx, w, h);

    const ias_range = 10;
    const alt_range = 50;
    draw_tape(ctx, w, h, ias, false, ias_range, 2);
    draw_tape(ctx, w, h, alt, true, alt_range, 10);
}
