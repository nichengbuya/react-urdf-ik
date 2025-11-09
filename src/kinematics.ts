import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

export type JointType = "revolute" | "continuous" | "prismatic";
export interface JointInfo {
  name: string;
  type: JointType;
  lower: number | null;  // revolute/continuous: rad, prismatic: m
  upper: number | null;
  node: any;             // urdf-loader joint node
}

export interface BuildOptions {
  urdfUrl: string;
  pkg?: Record<string, string>;
  baseLink: string;
  eeLink: string;
  radians?: boolean;
}

/** —— 解析 IK 所需几何参数（标准 DH，6R 球腕）——
 *  关节1~6的 DH 满足：a4≈0, a5≈0, a6≈0，腕三轴相交；常见于 ABB/KUKA/FANUC 标准手臂
 *  只需要前3轴的关键几何与末端法兰深度：
 *    d1: base->joint2 的竖直偏置（沿 z1）
 *    a2, a3: 2、3轴连杆长度（沿 x2/x3 的 a）
 *    d4: joint3 -> wrist center 的距离（沿 z3）
 *    d6: wrist center -> TCP 的距离（沿 z6，通常是法兰到工具的深度）
 *  注：若你有完整 DH，也可扩展；这里只取工业臂最常见的最小集
 */
export interface Analytic6RParams {
  d1: number;
  a2: number;
  a3: number;
  d4: number;
  d6: number;
  // 轴向约定：Rz-Ry-Ry-Rz-Ry-Rz（常见 ABB/KUKA）可通过左右手系、符号在使用处做轻度适配
  // 若你的机器人有特殊 α 符号（如 α2=±π/2），可在下面 R01/R12/R23 的构造里做小改
}

/** 解析 IK 的配置：关节限位、软约束、当前解的偏好 */
export interface AnalyticConfig {
  params: Analytic6RParams;
  // 关节限位（弧度/米），不传则仅用 URDF 的
  limits?: { lower: (number | null)[], upper: (number | null)[] };
  // 解选择：更偏好与当前 q 最近
  preferNearToQ?: boolean;
}

export default class URDFKinematics {
  public scene: THREE.Scene = new THREE.Scene();
  public robot!: URDFRobot;
  public joints: JointInfo[] = [];
  public base!: THREE.Object3D;
  public ee!: THREE.Object3D;
  public radians = true;
  public robotGroup: THREE.Group = new THREE.Group();

  // —— 运行态 —— //
  private _qLast: number[] | null = null;
  private _Tfiltered: THREE.Matrix4 | null = null;
  private _lastTQuat: THREE.Quaternion | null = null;
  private _jointRateLimit: number[] = [];   // rad/s or m/s
  private _colWeight: number[] = [];        // for DLS（腕轴降权）
  private _analyticCfg: AnalyticConfig | null = null;

  async build(opts: BuildOptions): Promise<void> {
    this.radians = opts.radians ?? true;
    const loader = new URDFLoader();
    const baseDir = opts.urdfUrl.replace(/[^/]*$/, "");
    // @ts-ignore
    loader.workingPath = baseDir;
    if (opts.pkg) loader.packages = opts.pkg;
    loader.manager.setURLModifier((u) => u);
    loader.manager.onError = (u) => console.error("[URDF asset load error]", u);
    // @ts-ignore
    loader.fetchOptions = { cache: "no-cache" };

    const robot = await new Promise<URDFRobot>((resolve, reject) => {
      loader.load(opts.urdfUrl, (rb) => resolve(rb as URDFRobot), undefined, reject);
    });

    this.scene = new THREE.Scene();
    this.robotGroup = new THREE.Group();
    // this.robotGroup// ROS(Z) → Three(Y)
    this.scene.add(this.robotGroup);

    this.robot = robot;
    this.robotGroup.add(this.robot);

    this.base = this.robot.getObjectByName(opts.baseLink) as THREE.Object3D;
    this.ee = this.robot.getObjectByName(opts.eeLink) as THREE.Object3D;
    if (!this.base) throw new Error(`Cannot find base link: ${opts.baseLink}`);
    if (!this.ee) throw new Error(`Cannot find ee link: ${opts.eeLink}`);

    const chain = collectChain(this.base, this.ee);
    if (!chain) throw new Error(`ee (${opts.eeLink}) is not a descendant of base (${opts.baseLink}).`);

    const joints: JointInfo[] = [];
    for (const node of chain) {
      // @ts-ignore
      const jt: string | undefined = node.jointType;
      if (!jt) continue;
      if (jt === "revolute" || jt === "continuous" || jt === "prismatic") {
        // @ts-ignore
        const lim = node.limit || {};
        const lower = toFiniteNumber(lim.lower);
        const upper = toFiniteNumber(lim.upper);
        joints.push({ name: node.name ?? "(unnamed)", type: jt as JointType, lower, upper, node });
      }
    }
    this.joints = joints;
    // 如果不是球腕，就彻底关掉解析 IK
    if (!isApproximatelySphericalWrist(this.joints)) {
      this._analyticCfg = null;
    }
    this._initJointRateLimit();
    this._initColumnWeight();

    this.scene.updateMatrixWorld(true);
    this._qLast = this.getQ();
  }

  /** 在外部（比如加载不同机器人后）调用，设置 6R 球腕解析参数 */
  setAnalyticConfig(cfg: AnalyticConfig | null) {
    this._analyticCfg = cfg;
  }

  private _initJointRateLimit() {
    const n = this.joints.length;
    this._jointRateLimit = new Array(n).fill(1);
    for (let i = 0; i < n; i++) {
      const J = this.joints[i];
      // @ts-ignore
      const v = Number(J?.node?.limit?.velocity);
      if (Number.isFinite(v) && v > 0) {
        this._jointRateLimit[i] = v;
      } else {
        if (J.type === "prismatic") this._jointRateLimit[i] = 0.4;
        else this._jointRateLimit[i] = (i >= 3 && n >= 6) ? 0.7 : 1.0; // 腕略慢
      }
    }
  }
  private _initColumnWeight() {
    const n = this.joints.length;
    this._colWeight = new Array(n).fill(1.0);
    for (let i = 0; i < n; i++) if (this.joints[i].type === "prismatic") this._colWeight[i] = 0.8;
    if (n >= 6) this._colWeight[3] = this._colWeight[4] = this._colWeight[5] = 0.6;
  }
  setQFromExternal(q: number[], opts?: { resetFilteredTarget?: boolean }) {
    this.setQ(q);                 // 正常把模型关节设置好
    this._qLast = q.slice();      // ★ 把当前姿态记录为 IK 的 seed

    if (opts?.resetFilteredTarget) {
      // 可选：把目标滤波状态重置到“当前 EE 位姿”，避免下一帧平滑把姿态又拉回去
      const Tcur = this.ee.matrixWorld.clone();
      this._Tfiltered = Tcur.clone();
      this._lastTQuat = new THREE.Quaternion().setFromRotationMatrix(Tcur);
    }
  }

  /** 把当前模型姿态作为 IK seed（不改姿态，仅同步种子） */
  seedFromCurrent() {
    this._qLast = this.getQ();
  }
  setQ(q: number[]) {
    if (q.length !== this.joints.length) throw new Error(`q size ${q.length} != DOF ${this.joints.length}`);
    for (let i = 0; i < q.length; i++) {
      const J = this.joints[i];
      let v = q[i];
      if (!this.radians && (J.type === "revolute" || J.type === "continuous"))
        v = THREE.MathUtils.degToRad(v);
      // @ts-ignore
      J.node.setJointValue(v);
    }
    this.scene.updateMatrixWorld(true);
  }
  getQ(): number[] {
    return this.joints.map((J) => {
      // @ts-ignore
      const v = Number(J.node.jointValue ?? 0);
      if (!this.radians && (J.type === "revolute" || J.type === "continuous"))
        return THREE.MathUtils.radToDeg(v);
      return v;
    });
  }
  fk(q: number[]): THREE.Matrix4 {
    this.setQ(q);
    return this.ee.matrixWorld.clone();
  }

  /* ====================  解析 IK（6R 球腕）  ==================== */

  /**
   * 返回候选解（弧度）数组：每个元素是长度=DOF的 q[]
   * 若不可解或模型不匹配，返回 []。
   *
   * 假设：标准 6R 球腕；Rz-Ry-Ry | Rz-Ry-Rz 的常见序列；
   * 使用几何法：先解腕中心，再 2R 平面解 θ2/θ3，θ1 由腕心方位，最后 R36 → θ4~θ6。
   */
  ikAnalytic6RSpherical(targetT: THREE.Matrix4): number[][] {
    if (!this._analyticCfg || this.joints.length < 6) return [];
    const { d1, a2, a3, d4, d6 } = this._analyticCfg.params;

    // 末端位置 & 姿态
    const p06 = new THREE.Vector3().setFromMatrixPosition(targetT);
    const R06 = new THREE.Quaternion().setFromRotationMatrix(targetT);
    const R06m = new THREE.Matrix4().makeRotationFromQuaternion(R06);

    // 计算腕中心（wrist center）：Pw = p06 - d6 * z6
    const z6 = new THREE.Vector3(0, 0, 1).applyMatrix4(R06m).sub(
      new THREE.Vector3(0, 0, 0).applyMatrix4(R06m)
    ); // 变换后的 z 轴方向
    z6.normalize();
    const Pw = p06.clone().addScaledVector(z6, -d6);

    // --- θ1：基座转角（绕 z1） ---
    const theta1_candidates: number[] = [];
    // 工业臂常见：θ1 = atan2(Pw.y, Pw.x)
    theta1_candidates.push(Math.atan2(Pw.y, Pw.x));
    // 某些有对称分支时，也可加一个 +π 分支；但通常一个即可
    // theta1_candidates.push(wrapToPi(theta1_candidates[0] + Math.PI));

    // --- θ2, θ3：用 joint2 平面（r, s）二维 2R 解 ---
    // 把 wrist center 投影到 joint2 平面：r 水平距离，s 垂直距离（相对 d1）
    const r = Math.hypot(Pw.x, Pw.y);
    const s = Pw.z - d1;

    // 考虑第3轴沿 z3 有一个 d4 到腕心（常见），所以连杆长度等效在 2R 解里为：L2=a2, L3=sqrt(a3^2 + d4^2)
    const L2 = a2;
    const L3 = Math.hypot(a3, d4);

    // 余弦定理
    const D = (r * r + s * s - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    if (D < -1 - 1e-6 || D > 1 + 1e-6) return []; // 超出可达
    const Dclamped = Math.min(1, Math.max(-1, D));
    const phi = Math.acos(Dclamped); // 关节3 的“肘”角

    // θ3 两支：肘上(+phi)、肘下(-phi)
    const theta3_candidates = [+phi, -phi];

    const solutions: number[][] = [];
    for (const th1 of theta1_candidates) {
      // base 坐标到 joint2 平面的相对角
      const beta = Math.atan2(s, r);

      for (const th3p of theta3_candidates) {
        // a3,d4 合成的等效角：gamma = atan2(d4, a3)
        const gamma = Math.atan2(d4, a3);

        // θ2 = beta - atan2(L3*sin(th3p), L2 + L3*cos(th3p)) - gamma
        const theta2 =
          beta - Math.atan2(L3 * Math.sin(th3p), L2 + L3 * Math.cos(th3p)) - gamma;

        // 真·θ3：等效 th3p 与机械臂 DH 的 θ3 可能差一个符号/偏移，这里按常见约定：
        // 令 θ3 = th3p  （如果你的机器人出现方向相反，可把这行改为 -th3p）
        const theta3 = th3p;

        // 现在构造 R03，并解出 R36 = R03^T R06，进而求 θ4~θ6
        const R01 = rotZ(th1);
        const R12 = rotY(theta2);
        const R23 = rotY(theta3);
        const R03 = new THREE.Matrix4().multiplyMatrices(new THREE.Matrix4().multiplyMatrices(R01, R12), R23);

        const R03_inv = new THREE.Matrix4().copy(R03).invert();
        const R36 = new THREE.Matrix4().multiplyMatrices(R03_inv, R06m);

        // wrist: Z-Y-Z（常见：关节4绕Z3, 5绕Y4, 6绕Z5），从 R36 提取欧拉
        const { ok, th4, th5, th6 } = decompose_ZYZ(R36);
        if (!ok) continue;

        const q = [th1, theta2, theta3, th4, th5, th6];
        solutions.push(q);
      }
    }

    // 关节限位过滤
    const limited = this._filterByLimits(solutions);
    return limited;
  }

  private _filterByLimits(cands: number[][]): number[][] {
    if (!cands.length) return cands;
    const n = this.joints.length;
    const lower = this.joints.map(j => j.lower ?? -Infinity);
    const upper = this.joints.map(j => j.upper ?? +Infinity);
    // 覆盖外部 limits
    if (this._analyticCfg?.limits) {
      const L = this._analyticCfg.limits.lower; const U = this._analyticCfg.limits.upper;
      if (L?.length === n) for (let i = 0; i < n; i++) if (L[i] != null) lower[i] = L[i] as number;
      if (U?.length === n) for (let i = 0; i < n; i++) if (U[i] != null) upper[i] = U[i] as number;
    }
    return cands.filter(q => q.every((v, i) => v >= lower[i] - 1e-9 && v <= upper[i] + 1e-9));
  }

  /* ====================  DLS 数值 IK（保留，上次稳定化版）  ==================== */

  ikDLS(
    targetT: THREE.Matrix4,
    q0: number[],
    opt?: {
      dt?: number; rateScale?: number; posTol?: number; oriTol?: number;
      posPhaseTol?: number; baseLambda?: number; step?: number; maxIter?: number;
    }
  ): { q: number[]; reached: boolean; iters: number; posErr: number; oriErr: number } {
    const dt = Math.max(1e-3, opt?.dt ?? 1 / 60);
    const rateScale = opt?.rateScale ?? 1;
    const posTol = opt?.posTol ?? 1e-4;
    const oriTol = opt?.oriTol ?? THREE.MathUtils.degToRad(0.3);
    const posPhaseTol = Math.max(posTol * 30, opt?.posPhaseTol ?? 3e-3);
    const baseLambda = opt?.baseLambda ?? 3e-2;
    const h0 = opt?.step ?? 2e-4;
    const maxIter = opt?.maxIter ?? 60;

    const perJointMaxDelta = this._jointRateLimit.map((v) => v * rateScale * dt);

    const q = q0.slice();
    const n = this.joints.length;

    const tTarget = new THREE.Vector3().setFromMatrixPosition(targetT);
    const Rtarget = new THREE.Quaternion().setFromRotationMatrix(targetT);
    const eePos = new THREE.Vector3();
    const eeQuat = new THREE.Quaternion();

    let reached = false;
    let finalPosErr = Infinity;
    let finalOriErr = Infinity;

    for (let iter = 0; iter < maxIter; iter++) {
      const T = this.fk(q);
      eePos.setFromMatrixPosition(T);
      eeQuat.setFromRotationMatrix(T);

      const posErrV = new THREE.Vector3().subVectors(tTarget, eePos);
      const posErr = posErrV.length();

      const doOri = posErr < posPhaseTol;

      const Rerr = mul(Rtarget, eeQuat.clone().invert());
      const oriErrV = quatLog(Rerr);
      const maxOriStep = 0.35;
      const len = oriErrV.length();
      if (len > maxOriStep) oriErrV.multiplyScalar(maxOriStep / Math.max(1e-9, len));
      const oriAngle = 2 * Math.acos(Math.min(1, Math.max(-1, Rerr.w)));

      const rotW = doOri ? 1.0 : 0.0;

      if (posErr < posTol && (doOri ? oriAngle < oriTol : true)) {
        reached = true;
        finalPosErr = posErr;
        finalOriErr = oriAngle;
        break;
      }

      const e = new Float64Array(6);
      e[0] = posErrV.x; e[1] = posErrV.y; e[2] = posErrV.z;
      e[3] = rotW * oriErrV.x; e[4] = rotW * oriErrV.y; e[5] = rotW * oriErrV.z;
      const eNorm = Math.hypot(e[0], e[1], e[2], e[3], e[4], e[5]);

      const lambda = baseLambda * (1 + 6 * Math.tanh(eNorm));
      const h = Math.min(5e-3, Math.max(2e-4, h0 * (1 + 0.15 * eNorm)));

      const J = new Float64Array(6 * n);
      const colNorm: number[] = new Array(n).fill(1);

      for (let j = 0; j < n; j++) {
        const bak = q[j];

        q[j] = bak + h;
        const Tp = this.fk(q);
        const pp = new THREE.Vector3().setFromMatrixPosition(Tp);
        const qp = new THREE.Quaternion().setFromRotationMatrix(Tp);

        q[j] = bak - h;
        const Tm = this.fk(q);
        const pm = new THREE.Vector3().setFromMatrixPosition(Tm);
        const qm = new THREE.Quaternion().setFromRotationMatrix(Tm);

        q[j] = bak;

        const dx = (pp.x - pm.x) / (2 * h);
        const dy = (pp.y - pm.y) / (2 * h);
        const dz = (pp.z - pm.z) / (2 * h);

        const dR = mul(qp, qm.clone().invert());
        const dqv = quatLog(dR);
        const rx = rotW * dqv.x / (2 * h);
        const ry = rotW * dqv.y / (2 * h);
        const rz = rotW * dqv.z / (2 * h);

        const wj = this._colWeight[j];
        J[0 * n + j] = dx * wj; J[1 * n + j] = dy * wj; J[2 * n + j] = dz * wj;
        J[3 * n + j] = rx * wj; J[4 * n + j] = ry * wj; J[5 * n + j] = rz * wj;

        colNorm[j] = Math.sqrt(dx * dx + dy * dy + dz * dz + rx * rx + ry * ry + rz * rz) || 1;
      }

      const H = new Float64Array(n * n).fill(0);
      const JT_e = new Float64Array(n).fill(0);
      for (let j = 0; j < n; j++) {
        for (let k = j; k < n; k++) {
          let s = 0; for (let r = 0; r < 6; r++) s += J[r * n + j] * J[r * n + k];
          H[j * n + k] = H[k * n + j] = s;
        }
        const sj = 1 / Math.max(1e-9, colNorm[j]);
        H[j * n + j] += (lambda * lambda) * (sj * sj);
      }
      for (let j = 0; j < n; j++) {
        let s = 0; for (let r = 0; r < 6; r++) s += J[r * n + j] * e[r];
        JT_e[j] = s;
      }

      const dq = solveSymmetric(H, JT_e, n);

      for (let j = 0; j < n; j++) {
        const md = perJointMaxDelta[j] ?? (0.04 * dt * 60);
        const step = THREE.MathUtils.clamp(dq[j], -md, md);
        q[j] += step;

        const { lower, upper, type } = this.joints[j];
        if (type === "revolute" || type === "continuous") {
          q[j] = wrapToPi(q[j]);
          if (type === "revolute") {
            if (lower !== null) q[j] = Math.max(q[j], lower);
            if (upper !== null) q[j] = Math.min(q[j], upper);
          }
        } else if (type === "prismatic") {
          if (lower !== null) q[j] = Math.max(q[j], lower);
          if (upper !== null) q[j] = Math.min(q[j], upper);
        }
      }

      finalPosErr = posErr;
      finalOriErr = oriAngle;
    }

    this.setQ(q);
    return { q, reached, iters: maxIter, posErr: finalPosErr, oriErr: finalOriErr };
  }

  /** 每帧调用：优先解析 IK，失败则回退 DLS */
  updateTowards(
    targetRawT: THREE.Matrix4,
    dt: number,
    opts?: { tauPos?: number; tauRot?: number; rateScale?: number; preferAnalytic?: boolean }
  ) {
    const Tf = this._smoothTarget(targetRawT, dt, opts?.tauPos ?? 0.02, opts?.tauRot ?? 0.02);
    const q0 = this._qLast ?? this.getQ();

    // 1) 尝试解析 IK（可关闭 preferAnalytic 强制数值）
    if ((opts?.preferAnalytic ?? true) && this._analyticCfg) {
      const cands = this.ikAnalytic6RSpherical(Tf);
      if (cands.length) {
        const best = pickNearest(cands, q0);
        this.setQ(best);
        this._qLast = best;
        return { q: best, reached: true, iters: 1, posErr: 0, oriErr: 0 };
      }
    }

    // 2) 回退 DLS
    const res = this.ikDLS(Tf, q0, { dt, rateScale: opts?.rateScale ?? 1 });
    this._qLast = res.q;
    return res;
  }

  private _smoothTarget(current: THREE.Matrix4, dt: number, tauPos = 0.02, tauRot = 0.02): THREE.Matrix4 {
    const alphaPos = 1 - Math.exp(-dt / Math.max(1e-3, tauPos));
    const alphaRot = 1 - Math.exp(-dt / Math.max(1e-3, tauRot));
    const p = new THREE.Vector3().setFromMatrixPosition(current);
    const R = new THREE.Quaternion().setFromRotationMatrix(current);
    if (!this._Tfiltered) {
      this._Tfiltered = current.clone();
      this._lastTQuat = R.clone();
      return this._Tfiltered;
    }
    const pf = new THREE.Vector3().setFromMatrixPosition(this._Tfiltered);
    const Rf = this._lastTQuat!.clone();
    if (Rf.dot(R) < 0) {
      R.x = -R.x; R.y = -R.y; R.z = -R.z; R.w = -R.w;
    }
    pf.lerp(p, alphaPos);
    Rf.slerp(R, alphaRot);
    const Tf = new THREE.Matrix4().makeRotationFromQuaternion(Rf);
    Tf.setPosition(pf);
    this._Tfiltered.copy(Tf);
    this._lastTQuat!.copy(Rf);
    return this._Tfiltered;
  }

    movejoint(
    target: number[],
    opts?: {
      speed?: number | number[];       // 目标关节速度上限（若缺省，用 URDF velocity 的 0.8 倍）
      smooth?: boolean;                // 是否余弦平滑启停
    }
  ): MoveHandle {
    const n = this.joints.length;
    if (!Array.isArray(target) || target.length !== n) {
      throw new Error(`movejoint: target size ${target?.length} != DOF ${n}`);
    }

    // 读取当前姿态作为起点
    const q0 = (this._qLast ?? this.getQ()).slice();
    const qT = target.slice();

    // 速度上限（逐关节）
    const base = this._jointRateLimit.map(v => v * 0.8); // 默认更保守
    let vmax: number[] = base.slice();
    if (typeof opts?.speed === 'number') vmax = new Array(n).fill(Math.max(1e-6, opts.speed));
    else if (Array.isArray(opts?.speed) && opts!.speed.length === n) {
      vmax = opts!.speed.map((v, i) => Math.max(1e-6, Math.min(v, base[i])));
    }

    // 计算最大持续时间（取所有关节的时间上界）
    const dq = qT.map((v, i) => wrapToPi(v - q0[i]));
    const ti = dq.map((d, i) => Math.abs(d) / Math.max(1e-6, vmax[i]));
    const T = Math.max(1e-6, ...ti);

    let t = 0;
    let canceled = false;
    const smooth = opts?.smooth ?? true;

    const update = (dt: number) => {
      if (canceled) return { done: true, q: this._qLast ?? this.getQ() };
      t = Math.min(T, t + Math.max(0, dt));
      // 进度参数 s in [0,1]
      let s = T <= 0 ? 1 : t / T;
      if (smooth) s = 0.5 - 0.5 * Math.cos(Math.PI * s); // 余弦启停
      const q = new Array(n);
      for (let i = 0; i < n; i++) q[i] = wrapToPi(q0[i] + dq[i] * s);

      this.setQ(q);
      this._qLast = q;

      const done = t >= T - 1e-9;
      return { done, q };
    };

    return {
      duration: T,
      update,
      cancel: () => { canceled = true; },
      progress: () => (T <= 0 ? 1 : Math.min(1, t / T)),
      kind: 'joint'
    };
  }

  /**
   * 笛卡尔直线运动：末端 TCP 沿直线匀速（平滑启停），姿态用 slerp 匀角速
   * target 可传 Matrix4，或 { position, quaternion/euler } 的对象
   * 速度：linear (m/s)，angularDeg (°/s) 可选；若只给 linear，则时长以线速度决定
   */
  movelinear(
    target: PoseLike,
    opts?: {
      linear?: number;         // m/s（默认 0.2）
      angularDeg?: number;     // °/s（默认 45）
      smooth?: boolean;        // 余弦启停
      rateScale?: number;      // 传给 updateTowards 的内部关节速率比例（默认 1）
      tauPos?: number;         // 目标滤波时间常数（降低颤动），默认 0.02
      tauRot?: number;         // 同上
      preferAnalytic?: boolean;// 优先解析 IK，默认 true
    }
  ): MoveHandle {
    const lin = Math.max(1e-6, opts?.linear ?? 0.2);
    const ang = Math.max(1e-6, (opts?.angularDeg ?? 45) * Math.PI / 180);
    const smooth = opts?.smooth ?? true;

    // 起点位姿（用当前 ee）
    const T0 = this.ee.matrixWorld.clone();
    const p0 = new THREE.Vector3().setFromMatrixPosition(T0);
    const q0 = new THREE.Quaternion().setFromRotationMatrix(T0);

    // 目标位姿规格化到 Matrix4
    const T1 = normalizePoseLikeToMat4(target);
    const p1 = new THREE.Vector3().setFromMatrixPosition(T1);
    const q1 = new THREE.Quaternion().setFromRotationMatrix(T1);

    // 距离与角距
    const L = p0.distanceTo(p1);
    let angDist = 2 * Math.acos(Math.min(1, Math.abs(q0.dot(q1))));
    // 角距离取 [0, π]
    angDist = Math.min(Math.PI, Math.max(0, angDist));

    const TLin = L / lin;
    const TAng = angDist / ang;
    const T = Math.max(1e-6, TLin, TAng);

    let t = 0;
    let canceled = false;

    const update = (dt: number) => {
      if (canceled) return { done: true, q: this._qLast ?? this.getQ() };
      t = Math.min(T, t + Math.max(0, dt));
      let s = T <= 0 ? 1 : t / T;
      if (smooth) s = 0.5 - 0.5 * Math.cos(Math.PI * s); // 平滑启停

      // 线性插值位置 + slerp插值姿态
      const p = p0.clone().lerp(p1, s);
      const q = q0.clone().slerp(q1, s);
      const Td = new THREE.Matrix4().makeRotationFromQuaternion(q);
      Td.setPosition(p);

      // 交给已有的 IK 跟踪器推进一次（用外部 dt）
      const res = this.updateTowards(Td, Math.max(1e-4, dt), {
        tauPos: opts?.tauPos ?? 0.02,
        tauRot: opts?.tauRot ?? 0.02,
        rateScale: opts?.rateScale ?? 1,
        preferAnalytic: opts?.preferAnalytic ?? true,
      });

      const done = t >= T - 1e-9;
      return { done, q: res.q };
    };

    return {
      duration: T,
      update,
      cancel: () => { canceled = true; },
      progress: () => (T <= 0 ? 1 : Math.min(1, t / T)),
      kind: 'linear'
    };
  }
}




/* ====== 解析 IK & DLS 的小工具 ====== */
/* ========= 小工具 & 类型 ========= */

export interface MoveHandle {
  /** 预计总时长（秒） */
  duration: number;
  /** 每帧调用推进；返回 { done, q }；done=true 表示运动完成 */
  update: (dt: number) => { done: boolean; q: number[] };
  /** 取消本次运动 */
  cancel: () => void;
  /** 0~1 的进度估计 */
  progress: () => number;
  /** 运动类型：'joint' | 'linear'（可用于上层 UI 展示） */
  kind: 'joint' | 'linear';
}

export type PoseLike =
  | THREE.Matrix4
  | {
      position?: THREE.Vector3 | [number, number, number];
      quaternion?: THREE.Quaternion | [number, number, number, number];
      euler?: [number, number, number]; // [roll, pitch, yaw] (rad)
      matrix?: number[];                // 长度16的列主序数组（可选）
    };

function normalizePoseLikeToMat4(p: PoseLike): THREE.Matrix4 {
  if (p instanceof THREE.Matrix4) return p.clone();

  if (p?.matrix && Array.isArray(p.matrix) && p.matrix.length === 16) {
    const m = new THREE.Matrix4();
    m.fromArray(p.matrix);
    return m;
  }
  const posArr = Array.isArray(p?.position) ? p!.position as number[] : null;
  const posVec = (p?.position instanceof THREE.Vector3) ? p.position as THREE.Vector3 : null;
  const eulerArr = Array.isArray(p?.euler) ? p!.euler as number[] : null;
  const quatArr  = Array.isArray(p?.quaternion) ? p!.quaternion as number[] : null;
  const quatObj  = (p?.quaternion instanceof THREE.Quaternion) ? p.quaternion as THREE.Quaternion : null;

  const T = new THREE.Matrix4();
  const q = new THREE.Quaternion();

  if (quatObj) q.copy(quatObj);
  else if (quatArr) q.set(quatArr[0], quatArr[1], quatArr[2], quatArr[3]).normalize();
  else if (eulerArr) {
    const e = new THREE.Euler(eulerArr[0], eulerArr[1], eulerArr[2], 'XYZ');
    q.setFromEuler(e);
  } else {
    q.identity();
  }
  T.makeRotationFromQuaternion(q);

  const t = new THREE.Vector3();
  if (posVec) t.copy(posVec);
  else if (posArr) t.set(posArr[0], posArr[1], posArr[2]);
  else t.set(0, 0, 0);

  T.setPosition(t);
  return T;
}

function isApproximatelySphericalWrist(urdfJoints: JointInfo[]): boolean {
  // 简单启发：检查 4、5、6 关节之间的连接是否几乎都是“纯旋转、无平移”
  // 这里用 urdf-loader 的 node.origin 取不方便，我们用已有的 URDF: 若 link_4->link_5 或 link_5->link_6 的 |xyz| 明显>几毫米，就不是球腕。
  // 实操：我们在 build 之后从 joints[i].node?.origin?.xyz 抓；如果取不到就保守返回 false
  try {
    const j5 = urdfJoints[4]?.node, j6 = urdfJoints[5]?.node;
    const t45 = j5?.origin?.xyz || [0, 0, 0];
    const t56 = j6?.origin?.xyz || [0, 0, 0];
    const L45 = Math.hypot(t45[0], t45[1], t45[2]);
    const L56 = Math.hypot(t56[0], t56[1], t56[2]);
    // 球腕通常 < 1~2mm，这里阈值给得宽松一点
    return L45 < 0.01 && L56 < 0.01;
  } catch { return false; }
}

function collectChain(base: THREE.Object3D, ee: THREE.Object3D): THREE.Object3D[] | null {
  const stack: THREE.Object3D[] = [];
  let cur: THREE.Object3D | null = ee;
  while (cur && cur !== base) { stack.push(cur); cur = cur.parent as THREE.Object3D | null; }
  if (cur !== base) return null;
  stack.reverse();
  return stack;
}
function mul(a: THREE.Quaternion, b: THREE.Quaternion): THREE.Quaternion {
  const q = new THREE.Quaternion();
  q.multiplyQuaternions(a, b);
  return q;
}
function quatLog(q: THREE.Quaternion): THREE.Vector3 {
  const v = new THREE.Vector3(q.x, q.y, q.z);
  const w = q.w;
  const nv = v.length();
  if (nv < 1e-12) return new THREE.Vector3(0, 0, 0);
  const ang = 2 * Math.atan2(nv, w);
  return v.multiplyScalar(ang / nv);
}
function solveSymmetric(H: Float64Array, b: Float64Array, n: number): number[] {
  const M = new Float64Array(n * (n + 1));
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) M[i * (n + 1) + j] = H[i * n + j];
    M[i * (n + 1) + n] = b[i];
  }
  for (let i = 0; i < n; i++) {
    let piv = i;
    for (let r = i + 1; r < n; r++)
      if (Math.abs(M[r * (n + 1) + i]) > Math.abs(M[piv * (n + 1) + i])) piv = r;
    if (piv !== i) {
      for (let c = i; c <= n; c++) {
        const tmp = M[i * (n + 1) + c];
        M[i * (n + 1) + c] = M[piv * (n + 1) + c];
        M[piv * (n + 1) + c] = tmp;
      }
    }
    const diag = M[i * (n + 1) + i] || 1e-12;
    for (let c = i; c <= n; c++) M[i * (n + 1) + c] /= diag;
    for (let r = 0; r < n; r++) if (r !== i) {
      const f = M[r * (n + 1) + i];
      for (let c = i; c <= n; c++) M[r * (n + 1) + c] -= f * M[i * (n + 1) + c];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = 0; i < n; i++) x[i] = M[i * (n + 1) + n];
  return x;
}
function wrapToPi(a: number): number {
  return ((a + Math.PI) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;
}
function rotZ(th: number): THREE.Matrix4 {
  const c = Math.cos(th), s = Math.sin(th);
  return new THREE.Matrix4().set(c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
}
function rotY(th: number): THREE.Matrix4 {
  const c = Math.cos(th), s = Math.sin(th);
  return new THREE.Matrix4().set(c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1);
}
/** 从 R36 提取 ZYZ 欧拉，对应 4(Z)-5(Y)-6(Z) 的球腕 */
function decompose_ZYZ(R: THREE.Matrix4): { ok: boolean; th4: number; th5: number; th6: number } {
  const m = R.elements;
  // R = Rz(th4) * Ry(th5) * Rz(th6)
  const r33 = m[10]; // (3,3)
  const th5 = Math.acos(Math.max(-1, Math.min(1, r33)));
  if (Number.isNaN(th5)) return { ok: false, th4: 0, th5: 0, th6: 0 };

  if (Math.abs(th5) < 1e-6) {
    // th5 ~ 0：万向节附近，合并到 th4+th6
    const th4 = Math.atan2(m[1], m[0]);     // atan2(r21, r11)
    const th6 = 0;
    return { ok: true, th4: wrapToPi(th4), th5: 0, th6: wrapToPi(th6) };
  } else if (Math.abs(th5 - Math.PI) < 1e-6) {
    const th4 = Math.atan2(-m[1], -m[0]);
    const th6 = 0;
    return { ok: true, th4: wrapToPi(th4), th5: Math.PI, th6: wrapToPi(th6) };
  } else {
    const th4 = Math.atan2(m[9], m[8]);     // atan2(r32, r31)
    const th6 = Math.atan2(m[6], -m[2]);    // atan2(r13, -r11) 需与上面定义匹配
    return { ok: true, th4: wrapToPi(th4), th5: th5, th6: wrapToPi(th6) };
  }
}
function pickNearest(cands: number[][], qRef: number[]): number[] {
  if (!cands.length) return qRef.slice();
  let best = cands[0], bestCost = Infinity;
  for (const q of cands) {
    let s = 0;
    for (let i = 0; i < q.length; i++) {
      const di = wrapToPi(q[i] - qRef[i]);
      s += di * di;
    }
    if (s < bestCost) { bestCost = s; best = q; }
  }
  return best;
}
function toFiniteNumber(v: any): number | null {
  if (v == null) return null;
  const n = typeof v === "number" ? v : parseFloat(String(v));
  return Number.isFinite(n) ? n : null;
}
