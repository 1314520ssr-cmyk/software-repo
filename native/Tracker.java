package com.motionlock.app;
import java.util.*;
/** 跟踪器：背景运动补偿 + AI 检测框关联 + 是否在移动的判断（纯 Java，不依赖安卓）。 */
final class Tracker {
static final class Det {
final float x, y, w, h, score; final String cls;
Det(float x, float y, float w, float h, String cls, float score) { this.x = x; this.y = y; this.w = w; this.h = h; this.cls = cls; this.score = score; }
}
static final class Trk {
int id, nd = 1, miss; float cx, cy, w, h, score, on, vx, vy; String cls = ""; boolean mv; long at;
final HashMap<String, Float> vote = new HashMap<>();
final ArrayList<double[]> hist = new ArrayList<>();
}
final int DW, DH, GW, GH;
private byte[] prev; private long prevT;
private double cumX, cumY, egoSp;
private final ArrayList<double[]> eh = new ArrayList<>();
private final ArrayList<Trk> tracks = new ArrayList<>();
private int nid = 1;
private Set<Integer> pins = new HashSet<>();
String status = "";
boolean egoOK;
private static final Set<String> SIM = new HashSet<>(Arrays.asList("car", "truck", "bus"));
private static final Set<String> ST = new HashSet<>(Arrays.asList("chair", "couch", "bench", "bed", "dining table", "potted plant", "tv", "refrigerator", "oven", "microwave", "sink", "toilet", "clock", "vase", "book", "laptop", "keyboard", "traffic light", "stop sign", "fire hydrant", "parking meter"));
Tracker(int dw, int dh, int gw, int gh) { DW = dw; DH = dh; GW = gw; GH = gh; }
synchronized void pin(int[] ids) { pins = new HashSet<>(); for (int i : ids) pins.add(i); }
/* ---------- 背景运动估计 ---------- */
private static final class Fit { double[] M; double ratio; }
private static List<double[]> flow(byte[] a, byte[] b, int w, int h, List<int[]> ex) {
final int BS = 8, R = 6; ArrayList<double[]> P = new ArrayList<>();
for (int y = R; y <= h - BS - R; y += 6) for (int x = R; x <= w - BS - R; x += 6) {
boolean skip = false;
for (int[] e : ex) if (x + 4 > e[0] && x + 4 < e[2] && y + 4 > e[1] && y + 4 < e[3]) { skip = true; break; }
if (skip) continue;
double m = 0; for (int j = 0; j < BS; j++) for (int i = 0; i < BS; i++) m += a[(y + j) * w + x + i] & 255; m /= 64;
double v = 0; for (int j = 0; j < BS; j++) for (int i = 0; i < BS; i++) { double e = (a[(y + j) * w + x + i] & 255) - m; v += e * e; }
if (v < 64 * 25) continue;
int best = sad(a, b, w, x, y, 0, 0, BS), bx = 0, by = 0;
for (int dy = -R; dy <= R; dy++) for (int dx = -R; dx <= R; dx++) {
if (dx == 0 && dy == 0) continue; int s = sad(a, b, w, x, y, dx, dy, BS);
if (s < best) { best = s; bx = dx; by = dy; }
}
if (best >= 64 * 14) continue;
double fx = 0, fy = 0;
if (bx > -R && bx < R && by > -R && by < R) {
int l = sad(a, b, w, x, y, bx - 1, by, BS), r = sad(a, b, w, x, y, bx + 1, by, BS), u = sad(a, b, w, x, y, bx, by - 1, BS), d = sad(a, b, w, x, y, bx, by + 1, BS);
double aa = l + r - 2.0 * best, cc = u + d - 2.0 * best;
if (aa > 1e-6) fx = Math.max(-.5, Math.min(.5, (l - r) / (2 * aa)));
if (cc > 1e-6) fy = Math.max(-.5, Math.min(.5, (u - d) / (2 * cc)));
}
P.add(new double[]{x + 4 - w / 2.0, y + 4 - h / 2.0, bx + fx, by + fy});
}
return P;
}
private static int sad(byte[] a, byte[] b, int w, int x, int y, int dx, int dy, int BS) {
int s = 0;
for (int j = 0; j < BS; j++) { int ra = (y + j) * w + x, rb = (y + j + dy) * w + x + dx; for (int i = 0; i < BS; i++) s += Math.abs((a[ra + i] & 255) - (b[rb + i] & 255)); }
return s;
}
private static double det3(double[][] m) {
return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
private static double[] ls(List<double[]> P) {
double n = P.size(), sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, d0 = 0, d1 = 0, d2 = 0, e0 = 0, e1 = 0, e2 = 0;
for (double[] q : P) { double x = q[0], y = q[1], u = q[2], v = q[3]; sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y; d0 += u; d1 += x * u; d2 += y * u; e0 += v; e1 += x * v; e2 += y * v; }
double[][] M = {{n, sx, sy}, {sx, sxx, sxy}, {sy, sxy, syy}};
double D = det3(M); if (Math.abs(D) < 1e-6) return null;
double[] out = new double[6]; double[][] rs = {{d0, d1, d2}, {e0, e1, e2}};
for (int k = 0; k < 2; k++) for (int c = 0; c < 3; c++) {
double[][] Q = new double[3][3];
for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Q[i][j] = (j == c) ? rs[k][i] : M[i][j];
out[k * 3 + c] = det3(Q) / D;
}
return out;
}
private static Fit fit(List<double[]> P) {
List<double[]> S = P; double[] M = null;
for (int it = 0; it < 5; it++) {
M = ls(S); if (M == null) return null;
double[] res = new double[P.size()];
for (int i = 0; i < res.length; i++) { double[] p = P.get(i); res[i] = Math.hypot(p[2] - (M[0] + M[1] * p[0] + M[2] * p[1]), p[3] - (M[3] + M[4] * p[0] + M[5] * p[1])); }
double[] so = res.clone(); Arrays.sort(so); double th = Math.max(.7, 2.5 * so[so.length >> 1]);
  ArrayList<double[]> S2 = new ArrayList<>(); for (int i = 0; i < res.length; i++) if (res[i] <= th) S2.add(P.get(i));
if (S2.size() < 6) break; S = S2;
}
Fit f = new Fit(); f.M = M; f.ratio = (double) S.size() / P.size(); return f;
}
/* ---------- 每个画面帧：更新背景位移，并让框跟着背景和速度走 ---------- */
synchronized void frame(byte[] g, long t, float gyro) {
if (prev == null) { prev = g; prevT = t; return; }
double dt = Math.max(.008, (t - prevT) / 1000.0), sx = (double) GW / DW, sy = (double) GH / DH, k = (double) DW / GW;
ArrayList<int[]> ex = new ArrayList<>();
for (Trk q : tracks) if (q.mv) ex.add(new int[]{(int) ((q.cx - q.w * .6) * sx), (int) ((q.cy - q.h * .6) * sy), (int) ((q.cx + q.w * .6) * sx), (int) ((q.cy + q.h * .6) * sy)});
List<double[]> P = flow(prev, g, GW, GH, ex);
Fit F = P.size() >= 8 ? fit(P) : null;
double[] E = null; String st = "";
if (gyro > 120) st = "注意：手机移动过快，暂停判定";
else if (F != null && F.ratio >= .45) E = F.M;
else if (F == null && gyro < 8) E = new double[6];
else st = "注意：背景对不上，暂停判定";
egoOK = E != null;
if (E != null) {
cumX += k * E[0]; cumY += k * E[3];
egoSp = egoSp * .7 + Math.hypot(k * E[0], k * E[3]) / dt * .3;
status = String.format(Locale.US, "背景位移 %.1f,%.1f", k * E[0], k * E[3]);
for (Trk q : tracks) {
double cx = q.cx - DW / 2.0, cy = q.cy - DH / 2.0;
q.cx += k * E[0] + E[1] * cx + E[2] * cy; q.cy += k * E[3] + E[4] * cx + E[5] * cy;
if (q.mv) { q.cx += q.vx * dt; q.cy += q.vy * dt; }
}
} else {
status = st; egoSp *= .7;
for (Trk q : tracks) { q.hist.clear(); q.on = Math.max(0, q.on - (float) dt); }
}
eh.add(new double[]{t, cumX, cumY});
while (eh.size() > 2 && t - eh.get(0)[0] > 3000) eh.remove(0);
prev = g; prevT = t;
}
private double[] cumAt(long tc) {
for (int i = eh.size() - 1; i >= 0; i--) if (eh.get(i)[0] <= tc) return new double[]{eh.get(i)[1], eh.get(i)[2]};
return eh.isEmpty() ? new double[]{cumX, cumY} : new double[]{eh.get(0)[1], eh.get(0)[2]};
}
private static double iou(double ax, double ay, double aw, double ah, double bx, double by, double bw, double bh) {
double x0 = Math.max(ax - aw / 2, bx - bw / 2), x1 = Math.min(ax + aw / 2, bx + bw / 2), y0 = Math.max(ay - ah / 2, by - bh / 2), y1 = Math.min(ay + ah / 2, by + bh / 2);
double i = Math.max(0, x1 - x0) * Math.max(0, y1 - y0); return i / (aw * ah + bw * bh - i + 1e-9);
}
/* ---------- 收到 AI 检测结果（tc 是这帧画面被拍下的时间）---------- */
synchronized void detections(List<Det> ds, long tc, long now) {
double[] c0 = cumAt(tc); double sx = cumX - c0[0], sy = cumY - c0[1], age = Math.max(0, (now - tc) / 1000.0);
int n0 = tracks.size(), nd = ds.size();
ArrayList<double[]> C = new ArrayList<>();
for (int ti = 0; ti < n0; ti++) {
Trk q = tracks.get(ti);
for (int j = 0; j < nd; j++) {
Det d = ds.get(j);
double dcx = d.x + d.w / 2 + sx + (q.mv ? q.vx * age : 0), dcy = d.y + d.h / 2 + sy + (q.mv ? q.vy * age : 0);
double io = iou(q.cx, q.cy, q.w, q.h, dcx, dcy, d.w, d.h), cd = Math.hypot(q.cx - dcx, q.cy - dcy) / Math.max(Math.max(d.w, d.h), Math.max(q.w, q.h));
if (io < .2 && cd > .5) continue;
double pen = (!q.cls.isEmpty() && !q.cls.equals(d.cls) && !(SIM.contains(q.cls) && SIM.contains(d.cls))) ? .5 : 0;
C.add(new double[]{1 - io + pen + cd * .3 - (pins.contains(q.id) ? .1 : 0), ti, j});
}
}
C.sort((p, q) -> Double.compare(p[0], q[0]));
boolean[] ut = new boolean[n0], ud = new boolean[nd];
for (double[] c : C) {
int ti = (int) c[1], j = (int) c[2];
if (c[0] > 1.1 || ut[ti] || ud[j]) continue;
ut[ti] = true; ud[j] = true;
Trk q = tracks.get(ti); Det d = ds.get(j);
double mx = d.x + d.w / 2, my = d.y + d.h / 2;
double nx = mx + sx + (q.mv ? q.vx * age : 0), ny = my + sy + (q.mv ? q.vy * age : 0);
q.w = (float) (q.w * .4 + d.w * .6); q.h = (float) (q.h * .4 + d.h * .6);
q.cx = (float) (q.cx * .4 + nx * .6); q.cy = (float) (q.cy * .4 + ny * .6);
for (Map.Entry<String, Float> e : q.vote.entrySet()) e.setValue(e.getValue() * .85f);
q.vote.merge(d.cls, d.score, Float::sum);
String bc = d.cls; float bv = -1; for (Map.Entry<String, Float> e : q.vote.entrySet()) if (e.getValue() > bv) { bv = e.getValue(); bc = e.getKey(); }
q.cls = bc; q.score = Math.max(q.score * .9f, d.score); q.nd++; q.miss = 0;
if (egoOK) { q.hist.add(new double[]{tc / 1000.0, mx - c0[0], my - c0[1]}); moveEval(q, now); }
}
for (int j = 0; j < nd; j++) {
Det d = ds.get(j); if (ud[j] || d.score < .4) continue;
double cx = d.x + d.w / 2 + sx, cy = d.y + d.h / 2 + sy; boolean dup = false;
for (Trk q : tracks) if (iou(q.cx, q.cy, q.w, q.h, cx, cy, d.w, d.h) > .5) { dup = true; break; }
if (dup) continue;
Trk q = new Trk(); q.id = nid++; q.cx = (float) cx; q.cy = (float) cy; q.w = d.w; q.h = d.h; q.cls = d.cls; q.score = d.score; q.vote.put(d.cls, d.score);
  if (egoOK) q.hist.add(new double[]{tc / 1000.0, d.x + d.w / 2 - c0[0], d.y + d.h / 2 - c0[1]});
tracks.add(q);
}
ArrayList<Trk> keep = new ArrayList<>();
for (int ti = 0; ti < tracks.size(); ti++) {
Trk q = tracks.get(ti);
if (ti < n0 && !ut[ti]) { q.miss++; if (q.miss >= (pins.contains(q.id) ? 25 : q.mv ? 10 : 6)) continue; }
if (q.cx < -DW * .3 || q.cx > DW * 1.3 || q.cy < -DH * .3 || q.cy > DH * 1.3) continue;
keep.add(q);
}
tracks.clear(); tracks.addAll(keep);
}
private void moveEval(Trk q, long now) {
ArrayList<double[]> h = q.hist;
while (h.size() > 2 && h.get(h.size() - 1)[0] - h.get(0)[0] > .9) h.remove(0);
int n = h.size(); double span = n > 1 ? h.get(n - 1)[0] - h.get(0)[0] : 0;
if (n < 4 || span < .35) return;
double t0 = h.get(0)[0], st = 0, stt = 0, sx = 0, sy = 0, sxt = 0, syt = 0;
for (double[] p : h) { double u = p[0] - t0; st += u; stt += u * u; sx += p[1]; sy += p[2]; sxt += p[1] * u; syt += p[2] * u; }
double den = n * stt - st * st; if (den < 1e-9) return;
double vx = (n * sxt - sx * st) / den, vy = (n * syt - sy * st) / den, sp = Math.hypot(vx, vy);
double thr = (Math.max(8, .1 * Math.max(q.w, q.h)) + .12 * egoSp) * (ST.contains(q.cls) ? 2 : 1);
if (sp > thr) q.on = Math.min(1f, q.on + .12f); else if (sp < thr * .5) q.on = Math.max(0f, q.on - .08f);
if (!q.mv && q.on >= .3f) q.mv = true; else if (q.mv && q.on <= 0f) q.mv = false;
if (q.mv) { q.at = now; q.vx = (float) vx; q.vy = (float) vy; } else { q.vx = 0; q.vy = 0; }
}
/** 当前所有可显示的框（拷贝）。 */
synchronized List<Trk> snapshot() {
ArrayList<Trk> out = new ArrayList<>();
for (Trk q : tracks) if (q.nd >= 2 || q.score >= .6f) {
Trk c = new Trk(); c.id = q.id; c.cx = q.cx; c.cy = q.cy; c.w = q.w; c.h = q.h; c.cls = q.cls; c.score = q.score; c.mv = q.mv; c.at = q.at; c.vx = q.vx; c.vy = q.vy; out.add(c);
}
return out;
}
}
