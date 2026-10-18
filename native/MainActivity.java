package com.motionlock.app;
import android.Manifest;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.RectF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Size;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.webkit.JavascriptInterface;
import android.webkit.WebView;
import androidx.camera.core.AspectRatio;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.Preview;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.core.content.ContextCompat;
import com.getcapacitor.BridgeActivity;
import com.google.mediapipe.framework.image.BitmapImageBuilder;
import com.google.mediapipe.framework.image.MPImage;
import com.google.mediapipe.tasks.components.containers.Category;
import com.google.mediapipe.tasks.components.containers.Detection;
import com.google.mediapipe.tasks.core.BaseOptions;
import com.google.mediapipe.tasks.core.Delegate;
import com.google.mediapipe.tasks.vision.core.RunningMode;
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetector;
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetectorResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
public class MainActivity extends BridgeActivity {
private static final int GW = 160, VW = 1280, VH = 720;
private PreviewView pv;
private WebView web;
private volatile Tracker tracker;
private volatile ObjectDetector detector;
private final AtomicBoolean detBusy = new AtomicBoolean(false);
private final AtomicInteger pend = new AtomicInteger(), detsA = new AtomicInteger();
private final ExecutorService exec = Executors.newSingleThreadExecutor(), exec2 = Executors.newSingleThreadExecutor();
private volatile String ai = "AI：加载中…";
private volatile float gyro = 0;
private volatile int[] pinIds = new int[0];
private int framesA = 0, aps = 0, dps = 0;
private long lastSec = 0, lastPush = 0;
private SensorManager sm;
private final SensorEventListener gl = new SensorEventListener() {
@Override public void onSensorChanged(SensorEvent e) { gyro = (float) (Math.sqrt(e.values[0] * e.values[0] + e.values[1] * e.values[1] + e.values[2] * e.values[2]) * 57.2958); }
@Override public void onAccuracyChanged(Sensor s, int a) { }
};
@Override public void onCreate(Bundle b) {
super.onCreate(b);
hide();
web = getBridge().getWebView();
web.setBackgroundColor(Color.TRANSPARENT);
pv = new PreviewView(this);
pv.setImplementationMode(PreviewView.ImplementationMode.COMPATIBLE);
pv.setScaleType(PreviewView.ScaleType.FILL_CENTER);
pv.setBackgroundColor(Color.BLACK);
((ViewGroup) web.getParent()).addView(pv, 0, new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
web.addJavascriptInterface(new JsApi(), "Android");
sm = (SensorManager) getSystemService(SENSOR_SERVICE);
Sensor g = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
if (g != null) sm.registerListener(gl, g, SensorManager.SENSOR_DELAY_GAME);
}
@Override public void onWindowFocusChanged(boolean f) { super.onWindowFocusChanged(f); if (f) hide(); }
@Override public void onDestroy() {
super.onDestroy();
try { sm.unregisterListener(gl); } catch (Throwable e) { }
try { if (detector != null) detector.close(); } catch (Throwable e) { }
exec.shutdownNow(); exec2.shutdownNow();
}
private void hide() {
getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
WindowManager.LayoutParams p = getWindow().getAttributes();
if (Build.VERSION.SDK_INT >= 28) p.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
p.preferredRefreshRate = 120f;
getWindow().setAttributes(p);
getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
}
private void js(String code) { runOnUiThread(() -> web.evaluateJavascript(code, null)); }
private static String q(String s) { return "'" + s.replace("\\", "\\\\").replace("'", "\\'").replace("\n", " ") + "'"; }
/* ---------- 网页界面调用的接口 ---------- */
public final class JsApi {
@JavascriptInterface public void start() { runOnUiThread(MainActivity.this::startCam); }
@JavascriptInterface public void pins(String csv) {
ArrayList<Integer> a = new ArrayList<>();
for (String s : csv.split(",")) { try { a.add(Integer.parseInt(s.trim())); } catch (Exception e) { } }
int[] ids = new int[a.size()]; for (int i = 0; i < ids.length; i++) ids[i] = a.get(i);
pinIds = ids; Tracker t = tracker; if (t != null) t.pin(ids);
}
@JavascriptInterface public void view(float tx, float ty, float s) {
final float d = getResources().getDisplayMetrics().density;
runOnUiThread(() -> {
pv.setPivotX(pv.getWidth() / 2f); pv.setPivotY(pv.getHeight() / 2f);
pv.setScaleX(s); pv.setScaleY(s); pv.setTranslationX(tx * d); pv.setTranslationY(ty * d);
});
}
}
/* ---------- 摄像头 ---------- */
private void startCam() {
if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) { requestPermissions(new String[]{Manifest.permission.CAMERA}, 77); return; }
if (detector == null) initDetector();
bindCam();
}
@Override public void onRequestPermissionsResult(int code, String[] perms, int[] res) {
super.onRequestPermissionsResult(code, perms, res);
if (code == 77) { if (res.length > 0 && res[0] == PackageManager.PERMISSION_GRANTED) startCam(); else js("nativeError('没有相机权限，请到系统设置里允许后重试')"); }
}
private void bindCam() {
final ListenableFutureHolder h = new ListenableFutureHolder();
h.f = ProcessCameraProvider.getInstance(this);
h.f.addListener(() -> {
try {
ProcessCameraProvider p = h.f.get();
Preview pr = new Preview.Builder().setTargetAspectRatio(AspectRatio.RATIO_16_9).build();
pr.setSurfaceProvider(pv.getSurfaceProvider());
ImageAnalysis an = new ImageAnalysis.Builder().setTargetResolution(new Size(640, 360)).setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST).setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888).build();
an.setAnalyzer(exec, this::analyze);
p.unbindAll();
p.bindToLifecycle(this, CameraSelector.DEFAULT_BACK_CAMERA, pr, an);
js("nativeReady()");
} catch (Throwable e) { js("nativeError(" + q("摄像头启动失败：" + e) + ")"); }
}, ContextCompat.getMainExecutor(this));
}
private static final class ListenableFutureHolder { com.google.common.util.concurrent.ListenableFuture<ProcessCameraProvider> f; }
/* ---------- AI 检测器 ---------- */
private ObjectDetector build(Delegate d) {
BaseOptions bo = BaseOptions.builder().setModelAssetPath("efficientdet_lite0.tflite").setDelegate(d).build();
ObjectDetector.ObjectDetectorOptions o = ObjectDetector.ObjectDetectorOptions.builder().setBaseOptions(bo).setRunningMode(RunningMode.LIVE_STREAM)
.setMaxResults(20).setScoreThreshold(0.35f).setResultListener(this::onDet).setErrorListener(e -> detBusy.set(false)).build();
return ObjectDetector.createFromOptions(this, o);
}
private void initDetector() {
exec2.execute(() -> {
String mode = "GPU";
ObjectDetector d;
try { d = build(Delegate.GPU); }
catch (Throwable e1) {
try { d = build(Delegate.CPU); mode = "CPU"; }
catch (Throwable e2) { ai = "AI不可用"; js("nativeError(" + q("AI 加载失败：" + e2) + ")"); return; }
}
detector = d; ai = "AI：就绪（" + mode + "）";
});
}
private void onDet(ObjectDetectorResult r, MPImage in) {
long tc = r.timestampMs(), now = SystemClock.elapsedRealtime();
ArrayList<Tracker.Det> ds = new ArrayList<>();
for (Detection d : r.detections()) {
if (d.categories().isEmpty()) continue;
RectF bb = d.boundingBox(); Category c = d.categories().get(0);
ds.add(new Tracker.Det(bb.left, bb.top, bb.width(), bb.height(), c.categoryName(), c.score()));
}
Tracker t = tracker;
if (t != null) t.detections(ds, tc, now);
detsA.incrementAndGet();
detBusy.set(false);
}
/* ---------- 每一帧画面 ---------- */
private void analyze(ImageProxy img) {
try {
long t = SystemClock.elapsedRealtime();
Bitmap bmp = img.toBitmap();
int rot = img.getImageInfo().getRotationDegrees();
if (rot != 0) { Matrix m = new Matrix(); m.postRotate(rot); bmp = Bitmap.createBitmap(bmp, 0, 0, bmp.getWidth(), bmp.getHeight(), m, true); }
int bw = bmp.getWidth(), bh = bmp.getHeight();
Tracker tk = tracker;
if (tk == null || tk.DW != bw || tk.DH != bh) { tk = new Tracker(bw, bh, GW, Math.max(8, Math.round(GW * bh / (float) bw))); tk.pin(pinIds); tracker = tk; }
Bitmap sm2 = Bitmap.createScaledBitmap(bmp, tk.GW, tk.GH, true);
int[] px = new int[tk.GW * tk.GH]; sm2.getPixels(px, 0, tk.GW, 0, 0, tk.GW, tk.GH);
byte[] g = new byte[px.length];
for (int i = 0; i < px.length; i++) { int c = px[i]; g[i] = (byte) ((((c >> 16) & 255) * 77 + ((c >> 8) & 255) * 150 + (c & 255) * 29) >> 8); }
tk.frame(g, t, gyro);
ObjectDetector d = detector;
if (d != null && detBusy.compareAndSet(false, true)) {
  try { d.detectAsync(new BitmapImageBuilder(bmp).build(), t); } catch (Throwable e) { detBusy.set(false); }
}
framesA++;
if (t - lastSec >= 1000) { aps = framesA; dps = detsA.getAndSet(0); framesA = 0; lastSec = t; }
push(tk, t);
} finally { img.close(); }
}
private void push(Tracker tk, long now) {
if (now - lastPush < 28 || pend.get() > 2) return;
lastPush = now;
StringBuilder sb = new StringBuilder(600);
sb.append("{\"st\":\"").append(tk.status).append("\",\"ai\":\"").append(ai).append("\",\"aps\":").append(aps).append(",\"dps\":").append(dps).append(",\"tr\":[");
List<Tracker.Trk> L = tk.snapshot(); double kx = VW / (double) tk.DW, ky = VH / (double) tk.DH;
for (int i = 0; i < L.size(); i++) {
Tracker.Trk q = L.get(i); if (i > 0) sb.append(',');
sb.append(String.format(Locale.US, "{\"id\":%d,\"x\":%.1f,\"y\":%.1f,\"w\":%.1f,\"h\":%.1f,\"c\":\"%s\",\"s\":%.2f,\"m\":%d,\"a\":%d,\"vx\":%.1f,\"vy\":%.1f}",
q.id, (q.cx - q.w / 2) * kx, (q.cy - q.h / 2) * ky, q.w * kx, q.h * ky, q.cls, q.score, q.mv ? 1 : 0, q.at > 0 ? (int) (now - q.at) : -1, q.vx * kx, q.vy * ky));
}
sb.append("]}");
final String code = "window.nativeTracks&&nativeTracks(" + sb + ")";
pend.incrementAndGet();
runOnUiThread(() -> web.evaluateJavascript(code, v -> pend.decrementAndGet()));
}
}
