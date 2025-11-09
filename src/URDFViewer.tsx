import React, { useEffect, useMemo, useRef, useState, useImperativeHandle, useCallback } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import URDFKinematics from "./kinematics";
THREE.Object3D.DEFAULT_UP = new THREE.Vector3( 0, 0, 1)
export type JointMeta = {
  name: string;
  type: "revolute" | "continuous" | "prismatic";
  unit: "deg" | "m";
  lower: number | null;
  upper: number | null;
};
export type URDFViewerHandle = {
  recenterHandle: () => void;
  // ★ 新增：双向控制 handle
  getHandlePose: () => {
    position: [number, number, number];
    euler: [number, number, number];       // 弧度 (XYZ)
    quaternion: [number, number, number, number];
    matrix: number[];                       // 4x4 列主序
  };
  setHandlePose: (pose: {
    position?: [number, number, number];
    eulerDeg?: [number, number, number];    // 用度数设置更直观
    eulerRad?: [number, number, number];
    quaternion?: [number, number, number, number];
  }) => void;
  startMoveJoint: (targetDeg: number[], opts?: { speed?: number | number[]; smooth?: boolean }) => void;
  startMoveLinear: (
    target: THREE.Matrix4 | {
      position?: THREE.Vector3 | [number, number, number];
      quaternion?: THREE.Quaternion | [number, number, number, number];
      euler?: [number, number, number]; // [roll,pitch,yaw] in rad
      matrix?: number[];
    },
    opts?: { linear?: number; angularDeg?: number; smooth?: boolean; rateScale?: number; tauPos?: number; tauRot?: number; preferAnalytic?: boolean }
  ) => void;
  cancelMotion: () => void;
};


type Props = {
  urdfUrl: string;
  baseLink: string;
  eeLink: string;
  qDeg?: number[];
  onJointsReady: (meta: JointMeta[]) => void;
  onQChange: (qRad: number[]) => void; // 仅在内部 IK/拖拽时回调
  onHandleChange?: (pose: {
    position: [number, number, number];
    eulerDeg: [number, number, number];
    quaternion: [number, number, number, number];
  }) => void;
};


const URDFViewer = React.forwardRef<URDFViewerHandle, Props>((props, ref) => {
  const { urdfUrl, baseLink, eeLink, qDeg, onJointsReady, onQChange } = props;
  const mountRef = useRef<HTMLDivElement>(null);
  const [info, setInfo] = useState("loading...");
  const kin = useMemo(() => new URDFKinematics(), []);

  const builtRef = useRef(false);
  const handleRef = useRef<THREE.Mesh | null>(null);
  const tctrlRef = useRef<TransformControls | null>(null);
  const orbitRef = useRef<OrbitControls | null>(null);

  const draggingRef = useRef(false);
  const latestTargetRef = useRef<THREE.Matrix4 | null>(null);
  const settleUntilRef = useRef<number>(0);
  const motionRef = useRef<ReturnType<URDFKinematics["movejoint"]> | ReturnType<URDFKinematics["movelinear"]> | null>(null);
  const pendingTargetDegRef = useRef<number[] | null>(null); // 等 build 后启动的关节运动
  const pendingLinearRef = useRef<{ target: any; opts?: any } | null>(null); // 等 build 后启动的直线运动

  const emitHandlePose = useCallback(() => {
    if (!handleRef.current || !props.onHandleChange) return;
    const h = handleRef.current;
    const p = h.position;
    const q = h.quaternion;
    const e = new THREE.Euler().setFromQuaternion(q, "XYZ");
    props.onHandleChange({
      position: [p.x, p.y, p.z],
      eulerDeg: [e.x, e.y, e.z].map(v => (v * 180 / Math.PI)) as [number, number, number],
      quaternion: [q.x, q.y, q.z, q.w],
    });
  }, [props]);

  const setHandlePoseInternal = useCallback((pose: {
    position?: [number, number, number];
    eulerDeg?: [number, number, number];
    eulerRad?: [number, number, number];
    quaternion?: [number, number, number, number];
  }) => {
    if (!handleRef.current) return;
    const h = handleRef.current;

    if (pose.position) {
      const [x, y, z] = pose.position;
      h.position.set(x, y, z);
    }
    if (pose.quaternion) {
      const [qx, qy, qz, qw] = pose.quaternion;
      h.quaternion.set(qx, qy, qz, qw).normalize();
    } else if (pose.eulerRad) {
      const [rx, ry, rz] = pose.eulerRad;
      h.quaternion.setFromEuler(new THREE.Euler(rx, ry, rz, "XYZ"));
    } else if (pose.eulerDeg) {
      const [rx, ry, rz] = pose.eulerDeg.map(d => d * Math.PI / 180) as [number, number, number];
      h.quaternion.setFromEuler(new THREE.Euler(rx, ry, rz, "XYZ"));
    }

    h.updateMatrixWorld(true);
    latestTargetRef.current = h.matrixWorld.clone(); // 让 IK 目标也同步
    settleUntilRef.current = performance.now() + 250;  // 200~300ms 都可以
    emitHandlePose();
  }, [emitHandlePose]);

  useImperativeHandle(ref, () => ({
    recenterHandle() {
      if (!builtRef.current || !handleRef.current) return;
      const q = kin.getQ();
      const T = kin.fk(q);
      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      const scl = new THREE.Vector3();
      T.decompose(pos, quat, scl);
      handleRef.current.position.copy(pos);
      handleRef.current.quaternion.copy(quat);
      handleRef.current.updateMatrixWorld(true);
      latestTargetRef.current = handleRef.current.matrixWorld.clone();
      emitHandlePose();
    },
    getHandlePose() {
      const h = handleRef.current!;
      const e = new THREE.Euler().setFromQuaternion(h.quaternion, "XYZ");
      return {
        position: [h.position.x, h.position.y, h.position.z] as [number, number, number],
        euler: [e.x, e.y, e.z] as [number, number, number],
        quaternion: [h.quaternion.x, h.quaternion.y, h.quaternion.z, h.quaternion.w] as [number, number, number, number],
        matrix: h.matrixWorld.toArray(),
      };
    },
    setHandlePose(pose) {
      setHandlePoseInternal(pose);
    },
    startMoveJoint(targetDeg, opts) {
      if (!builtRef.current) { pendingTargetDegRef.current = targetDeg.slice(); return; }
      const targetRad = targetDeg.map((v, i) =>
        kin.joints[i]?.type === "prismatic" ? v : (v * Math.PI / 180)
      );
      motionRef.current = kin.movejoint(targetRad, { speed: opts?.speed, smooth: opts?.smooth });
    },
    startMoveLinear(target, opts) {
      if (!builtRef.current) { pendingLinearRef.current = { target, opts }; return; }
      motionRef.current = kin.movelinear(target, opts);
    },
    cancelMotion() {
      motionRef.current?.cancel();
      motionRef.current = null;
    },
  }));




  // ★ 外部 qDeg 变动时，只同步到机器人，不再 onQChange 回传，防止回声环
  useEffect(() => {
    if (!builtRef.current || !qDeg || qDeg.length !== kin.joints.length) return;
    try {
      const qRad = qDeg.map((v, i) => {
        const J = kin.joints[i];
        return J?.type === "prismatic" ? v : (v * Math.PI / 180);
      });

      // 只有真的变化才同步，避免无意义刷新
      const cur = kin.getQ();
      let changed = qRad.length !== cur.length;
      if (!changed) for (let i = 0; i < qRad.length; i++) if (Math.abs(qRad[i] - cur[i]) > 1e-6) { changed = true; break; }
      if (!changed) return;

      // ★ 用“外部设姿”同步，并把 IK seed 更新为这个姿态
      kin.setQFromExternal(qRad, {
        // 是否重置目标平滑器为当前 EE（你自己取舍，抖时开 true 更稳）
        resetFilteredTarget: false,
      });

      // 若当前没在拖拽，把 handle 吸到新的 FK 末端，视觉一致
      if (!draggingRef.current && handleRef.current) {
        const T = kin.fk(qRad);
        const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
        T.decompose(pos, quat, scl);
        handleRef.current.position.copy(pos);
        handleRef.current.quaternion.copy(quat);
        handleRef.current.updateMatrixWorld(true);
        emitHandlePose();
      }
    } catch { /* ignore */ }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [qDeg]);

  useEffect(() => {
    if (!mountRef.current) return;

    const r = new THREE.WebGLRenderer({ antialias: true });
    const pr = Math.min(window.devicePixelRatio || 1, 2);
    r.setPixelRatio(pr);
    r.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(r.domElement);

    const cam = new THREE.PerspectiveCamera(50, mountRef.current.clientWidth / mountRef.current.clientHeight, 0.01, 200);
    cam.position.set(1.6, -2.2, 1.1);

    const orbit = new OrbitControls(cam, r.domElement);
    orbit.enableDamping = true;
    orbit.dampingFactor = 0.12;
    orbit.target.set(0, 0.4, 0);
    orbit.update();
    orbitRef.current = orbit;

    const tctrl = new TransformControls(cam, r.domElement);
    tctrl.setMode("translate");
    tctrlRef.current = tctrl;

    (r.domElement as HTMLCanvasElement).style.touchAction = "none";
    const shouldIgnoreKey = (e: KeyboardEvent) => {
      const el = e.target as HTMLElement | null;
      if (!el) return false;
      const tag = el.tagName?.toLowerCase();
      const isForm = tag === "input" || tag === "textarea" || tag === "select";
      const editable = (el as any).isContentEditable === true;
      return isForm || editable;
    };

    const onKey = (e: KeyboardEvent) => {
      if (shouldIgnoreKey(e)) return;
      if (!tctrlRef.current) return;

      const tctrl = tctrlRef.current;

      if (e.key === "t" || e.key === "T") {
        tctrl.setMode("translate");
        e.preventDefault();
      } else if (e.key === "r" || e.key === "R") {
        tctrl.setMode("rotate");
        e.preventDefault();
      } else if (e.key === "w" || e.key === "W") {
        tctrl.setSpace("world");
        e.preventDefault();
      } else if (e.key === "e" || e.key === "E") {
        tctrl.setSpace("local");
        e.preventDefault();
      }
    };
    window.addEventListener("keydown", onKey);
    let stop = false;
    let last = performance.now();
    const loop = () => {
      if (stop) return;
      const now = performance.now();
      const dt = Math.min(0.033, Math.max(0.001, (now - last) / 1000));
      last = now;

      const needSettle = now < settleUntilRef.current;
      if (motionRef.current) {
        const res = motionRef.current.update(dt);
        onQChange(res.q);
        if (res.done) {
          // 运动完成后，同步 handle 到最新 FK，保证视觉一致
          motionRef.current = null;
          if (handleRef.current) {
            const T = kin.fk(res.q);
            const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3();
            T.decompose(p, q, s);
            handleRef.current.position.copy(p);
            handleRef.current.quaternion.copy(q);
            handleRef.current.updateMatrixWorld(true);
            latestTargetRef.current = handleRef.current.matrixWorld.clone();
             emitHandlePose();
          }
        }
      } else {
        // 2) 没有运动时，才根据拖拽/缓和去 IK
        const shouldSolve = draggingRef.current || needSettle;
        if (shouldSolve && latestTargetRef.current) {
          const res = kin.updateTowards(latestTargetRef.current, dt, {
            tauPos: 0.001,
            tauRot: 0.001,
            rateScale: 1.0,
          });
          onQChange(res.q);
        }
      }

      orbitRef.current?.update();
      r.render(kin.scene, cam);
      requestAnimationFrame(loop);
    };

    const onResize = () => {
      if (!mountRef.current) return;
      cam.aspect = mountRef.current.clientWidth / mountRef.current.clientHeight;
      cam.updateProjectionMatrix();
      r.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    };
    window.addEventListener("resize", onResize);

    (async () => {
      try {
        // 省略 build 细节…（保持你原先的）
        await kin.build({ urdfUrl, baseLink, eeLink, radians: true, pkg: derivePkgMap(urdfUrl) });
        if (pendingTargetDegRef.current) {
          const tmp = pendingTargetDegRef.current.slice();
          pendingTargetDegRef.current = null;
          motionRef.current = kin.movejoint(
            tmp.map((v, i) => kin.joints[i]?.type === "prismatic" ? v : (v * Math.PI / 180)),
            { speed: 0.8, smooth: true }
          );
        }
        // 若有挂起的直线运动，立即启动
        if (pendingLinearRef.current) {
          const { target, opts } = pendingLinearRef.current;
          pendingLinearRef.current = null;
          motionRef.current = kin.movelinear(target, opts);
        }
        onJointsReady(
          kin.joints.map((j: any) => ({
            name: j.name,
            type: j.type,
            unit: j.type === "prismatic" ? "m" : "deg",
            lower: j.type === "prismatic" ? j.lower : (j.lower == null ? null : (j.lower * 180 / Math.PI)),
            upper: j.type === "prismatic" ? j.upper : (j.upper == null ? null : (j.upper * 180 / Math.PI)),
          }))
        );

        // 灯光/网格/handle 初始化（同你原来的）
        const scene = kin.scene;
        scene.add(new THREE.AmbientLight(0xffffff, 0.6));
        const dl = new THREE.DirectionalLight(0xffffff, 0.8);
        dl.position.set(3, 5, 2);
        scene.add(dl);
        const gridHelper = new THREE.GridHelper(4, 40, 0x888888, 0x444444);
        gridHelper.rotation.x = -Math.PI / 2; 

        scene.add(gridHelper);

        const handle = new THREE.Mesh(
          new THREE.SphereGeometry(0.03, 12, 12),
          new THREE.MeshStandardMaterial({ metalness: 0.2, roughness: 0.6 })
        );
        handleRef.current = handle;
        scene.add(handle);
        scene.add(tctrl.getHelper());
        tctrl.attach(handle);

        // 初始 FK 放 handle
        const q0 = kin.joints.map(() => 0);
        kin.setQ(q0);
        const T0 = kin.fk(q0);
        {
          const p = new THREE.Vector3(); const q = new THREE.Quaternion(); const s = new THREE.Vector3();
          T0.decompose(p, q, s);
          handle.position.copy(p);
          handle.quaternion.copy(q);
          handle.updateMatrixWorld(true);
          latestTargetRef.current = handle.matrixWorld.clone();
          emitHandlePose();
        }

        // 拖拽时更新 target
        tctrl.addEventListener("change", () => {
          if (!handleRef.current) return;
          if (draggingRef.current) latestTargetRef.current = handleRef.current.matrixWorld.clone();
           emitHandlePose();
        });
        tctrl.addEventListener("dragging-changed", (e: any) => {
          const dragging = !!e.value;
          draggingRef.current = dragging;
          orbit.enabled = !dragging;
          if (dragging) {
            if (handleRef.current) latestTargetRef.current = handleRef.current.matrixWorld.clone();
          } else {
            settleUntilRef.current = performance.now() + 220;
            if (handleRef.current) {
              const q = kin.getQ();
              const T = kin.fk(q);
              const p = new THREE.Vector3(); const qb = new THREE.Quaternion(); const s = new THREE.Vector3();
              T.decompose(p, qb, s);
              handleRef.current.position.copy(p);
              handleRef.current.quaternion.copy(qb);
              handleRef.current.updateMatrixWorld(true);
               emitHandlePose();
            }
          }
        });

        builtRef.current = true;
        setInfo(`Loaded DOF=${kin.joints.length}`);
        requestAnimationFrame(loop);
      } catch (e: any) {
        console.error(e);
        setInfo("Load error: " + e?.message);
      }
    })();

    return () => {
      stop = true;
      window.removeEventListener("resize", onResize);
      r.dispose();
      if (r.domElement.parentElement) r.domElement.parentElement.removeChild(r.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [urdfUrl, baseLink, eeLink, kin]);

  return (
    <>
      <div ref={mountRef} style={{ position: "absolute", inset: 0 }} />
      <div style={{ position: "absolute", left: 12, top: 12, color: "#fff", font: "12px/1.4 ui-monospace,monospace", pointerEvents: "none", textShadow: "0 1px 2px rgba(0,0,0,.6)" }}>
        {info}
      </div>
      <div style={{ position: "absolute", right: 12, top: 12, color: "#fff", font: "12px/1.4 ui-monospace,monospace", pointerEvents: "none", textShadow: "0 1px 2px rgba(0,0,0,.6)" }}>
        {'按T/R 切换平移/旋转'}
      </div>
    </>
  );
});

export default URDFViewer;
function derivePkgMap(urdfUrl: string): Record<string, string> {
  // urdfUrl: /robots/ros-industrial/abb/abb_irb2400_support/urdf/irb2400.urdf
  const m = urdfUrl.match(/^(.*\/)([^/]+)\/urdf\/([^/]+)\.urdf$/);
  if (!m) return {};
  const baseDir = m[1];           // /robots/ros-industrial/abb/
  const pkgName = m[2];           // abb_irb2400_support
  const model = m[3];           // irb2400
  const pkgRoot = `${baseDir}${pkgName}/`;

  // 三个“包名”都映射：主包 + visual + collision
  return {
    [pkgName]: pkgRoot,
    visual: `${pkgRoot}meshes/${model}/visual/`,
    collision: `${pkgRoot}meshes/${model}/collision/`,
  };
}
