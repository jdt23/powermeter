import SwiftUI
import Combine

struct BikeComputerView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @EnvironmentObject var connectivityManager: PhoneConnectivityManager

    var onWorkoutEnd: () -> Void

    @State private var showingStopConfirmation = false
    @State private var showingSettings = false
    @State private var recordingCancellable: AnyCancellable?
    @State private var showDetail = false
    @AppStorage("ftp") private var ftp: Double = 200.0

    private var on: Bool { workoutSession.state != .idle }

    var body: some View {
        // Dead simple: VStack, fixed status+controls, everything else flexible
        VStack(spacing: 0) {
            // Status bar
            HStack(spacing: 4) {
                Circle().fill(connColor).frame(width: 6, height: 6)
                Text(bleManager.connectionState.rawValue).font(.system(size: 9)).foregroundColor(.gray)
                if connectivityManager.isWatchReachable {
                    Image(systemName: "applewatch.radiowaves.left.and.right").font(.system(size: 8)).foregroundColor(.green)
                }
                Spacer()
                if on {
                    Text(fmtTime(workoutSession.elapsed))
                        .font(.system(size: 18, weight: .bold, design: .monospaced)).foregroundColor(.white)
                } else {
                    Button(action: { showingSettings = true }) {
                        Image(systemName: "gearshape.fill").font(.system(size: 14)).foregroundColor(.gray)
                    }
                }
            }
            .padding(.horizontal, 10)
            .frame(maxWidth: .infinity)
            .frame(height: 28)

            if !showDetail {
                // PAGE 1: Main metrics
                mainMetrics
            } else {
                // PAGE 2: Detail metrics
                detailMetrics
            }

            // Controls
            HStack(spacing: 6) {
                if !on {
                    Button(action: startWorkout) {
                        Label("Start Workout", systemImage: "play.fill").font(.system(size: 14, weight: .bold))
                            .frame(maxWidth: .infinity, maxHeight: .infinity).background(Color.green).foregroundColor(.black).cornerRadius(8)
                    }
                } else {
                    if showDetail {
                        Button(action: { showDetail = false }) {
                            Image(systemName: "chevron.left").font(.system(size: 16, weight: .bold))
                                .frame(width: 44, maxHeight: .infinity).background(Color.white.opacity(0.1)).foregroundColor(.white).cornerRadius(8)
                        }
                    }
                    if workoutSession.state == .active {
                        Button(action: pauseWorkout) {
                            Image(systemName: "pause.fill").font(.system(size: 16, weight: .bold))
                                .frame(maxWidth: .infinity, maxHeight: .infinity).background(Color.white.opacity(0.1)).foregroundColor(.white).cornerRadius(8)
                        }
                    } else {
                        Button(action: resumeWorkout) {
                            Image(systemName: "play.fill").font(.system(size: 16, weight: .bold))
                                .frame(maxWidth: .infinity, maxHeight: .infinity).background(Color.green).foregroundColor(.black).cornerRadius(8)
                        }
                    }
                    if !showDetail {
                        Button(action: { showDetail = true }) {
                            Image(systemName: "chevron.right").font(.system(size: 16, weight: .bold))
                                .frame(width: 44, maxHeight: .infinity).background(Color.white.opacity(0.1)).foregroundColor(.white).cornerRadius(8)
                        }
                    }
                    Button(action: { showingStopConfirmation = true }) {
                        Image(systemName: "stop.fill").font(.system(size: 16, weight: .bold))
                            .frame(width: 54, maxHeight: .infinity).background(Color.red.opacity(0.8)).foregroundColor(.white).cornerRadius(8)
                    }
                }
            }
            .padding(.horizontal, 2)
            .frame(maxWidth: .infinity)
            .frame(height: 44)
        }
        .background(Color.black)
        .onAppear { startRecording(); workoutSession.ftp = ftp }
        .onDisappear { recordingCancellable?.cancel() }
        .confirmationDialog("End Workout?", isPresented: $showingStopConfirmation) {
            Button("End Workout", role: .destructive) { endWorkout() }
            Button("Cancel", role: .cancel) {}
        }
        .sheet(isPresented: $showingSettings) { SettingsView() }
    }

    // MARK: - Main Metrics (Page 1)

    private var mainMetrics: some View {
        VStack(spacing: 2) {
            // Power
            pwrCard

            // Cadence | Resistance
            HStack(spacing: 2) {
                tile("CADENCE", "\(bleManager.cadence)", "RPM", .cyan,
                     on ? "avg \(Int(workoutSession.averageCadence))" : nil,
                     on && workoutSession.maxCadence > 0 ? "max \(Int(workoutSession.maxCadence))" : nil)
                tile("RESISTANCE", "\(bleManager.resistance)", "", .orange,
                     on ? "avg \(Int(workoutSession.averageResistance))" : nil,
                     on && workoutSession.maxResistance > 0 ? "max \(Int(workoutSession.maxResistance))" : nil)
            }

            // HR | Speed
            HStack(spacing: 2) {
                tile("HEART RATE", currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--", "BPM", .red,
                     on && workoutSession.averageHeartRate > 0 ? "avg \(Int(workoutSession.averageHeartRate))" : nil,
                     on && workoutSession.maxHeartRate > 0 ? "max \(Int(workoutSession.maxHeartRate))" : nil)
                tile("SPEED", on ? String(format: "%.1f", workoutSession.speed) : "--", "MPH", .mint,
                     on && workoutSession.averageSpeed > 0 ? String(format: "avg %.1f", workoutSession.averageSpeed) : nil,
                     on && workoutSession.maxSpeed > 0 ? String(format: "max %.1f", workoutSession.maxSpeed) : nil)
            }

            // Distance | Calories
            HStack(spacing: 2) {
                tile("DISTANCE", on ? String(format: "%.2f", workoutSession.distance) : "--", "MI", .mint, nil, nil)
                tile("CALORIES", "\(Int(workoutSession.totalCalories))", "KCAL", .yellow, nil, nil)
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    // MARK: - Detail Metrics (Page 2)

    private var detailMetrics: some View {
        VStack(spacing: 2) {
            HStack(spacing: 2) {
                tile("NORMALIZED PWR", "\(Int(workoutSession.normalizedPower))", "W", .purple, nil, nil)
                tile("INTENSITY", String(format: "%.2f", workoutSession.intensityFactor), "IF", .indigo, nil, nil)
            }
            HStack(spacing: 2) {
                tile("TRAINING STRESS", "\(Int(workoutSession.tss))", "TSS", .pink, nil, nil)
                zoneTile
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    // MARK: - Power Card

    private var pwrCard: some View {
        let z = workoutSession.powerZone; let c = on ? zColor(z) : pwrColor
        return ZStack {
            RoundedRectangle(cornerRadius: 8)
                .fill(LinearGradient(colors: [c.opacity(0.15), Color.white.opacity(0.02)], startPoint: .top, endPoint: .bottom))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(c.opacity(0.3), lineWidth: 1))
            VStack {
                HStack {
                    Text("POWER").font(.system(size: 9, weight: .bold)).foregroundColor(c.opacity(0.7)).tracking(2)
                    Spacer()
                    if on {
                        Text("Z\(z.rawValue) \(z.label)").font(.system(size: 9, weight: .bold)).foregroundColor(c.opacity(0.8))
                            .padding(.horizontal, 6).padding(.vertical, 1).background(Capsule().fill(c.opacity(0.15)))
                    }
                }.padding(.horizontal, 8)
                Spacer(minLength: 0)
                HStack(alignment: .lastTextBaseline, spacing: 3) {
                    Text("\(bleManager.power)")
                        .font(.system(size: 80, weight: .heavy, design: .rounded))
                        .foregroundColor(.white).minimumScaleFactor(0.3).lineLimit(1)
                    Text("W").font(.system(size: 18, weight: .semibold)).foregroundColor(Color.white.opacity(0.4))
                }
                Spacer(minLength: 0)
                if on {
                    HStack(spacing: 14) {
                        Text("avg \(Int(workoutSession.averagePower))").font(.system(size: 10, weight: .semibold, design: .rounded))
                        Text("max \(Int(workoutSession.maxPower))").font(.system(size: 10, weight: .semibold, design: .rounded))
                    }.foregroundColor(c.opacity(0.45)).padding(.bottom, 2)
                }
            }.padding(.top, 4)
        }
    }

    // MARK: - Zone Tile

    private var zoneTile: some View {
        let z = workoutSession.powerZone; let c = zColor(z)
        return ZStack {
            RoundedRectangle(cornerRadius: 8).fill(c.opacity(0.1))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(c.opacity(0.3), lineWidth: 1))
            VStack {
                Text("ZONE").font(.system(size: 9, weight: .bold)).foregroundColor(c.opacity(0.65)).tracking(1)
                Spacer(minLength: 0)
                Text("Z\(z.rawValue)").font(.system(size: 60, weight: .heavy, design: .rounded)).foregroundColor(c)
                    .minimumScaleFactor(0.3).lineLimit(1)
                Text(z.label).font(.system(size: 16, weight: .bold)).foregroundColor(c.opacity(0.7))
                Spacer(minLength: 0)
            }.padding(.top, 4)
        }
    }

    // MARK: - Tile

    private func tile(_ label: String, _ val: String, _ unit: String, _ color: Color,
                      _ avg: String?, _ peak: String?) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 8).fill(Color.white.opacity(0.035))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(color.opacity(0.15), lineWidth: 1))
            VStack {
                Text(label).font(.system(size: 9, weight: .bold)).foregroundColor(color.opacity(0.65)).tracking(1)
                    .lineLimit(1).minimumScaleFactor(0.7)
                Spacer(minLength: 0)
                HStack(alignment: .lastTextBaseline, spacing: 2) {
                    Text(val).font(.system(size: 40, weight: .bold, design: .rounded))
                        .foregroundColor(.white).minimumScaleFactor(0.3).lineLimit(1)
                    if !unit.isEmpty {
                        Text(unit).font(.system(size: 9, weight: .medium)).foregroundColor(Color.white.opacity(0.3))
                    }
                }
                Spacer(minLength: 0)
                if avg != nil || peak != nil {
                    HStack(spacing: 4) {
                        if let a = avg { Text(a).foregroundColor(color.opacity(0.4)) }
                        if let p = peak { Text(p).foregroundColor(color.opacity(0.4)) }
                    }.font(.system(size: 9, weight: .semibold, design: .rounded)).lineLimit(1).minimumScaleFactor(0.7)
                }
            }.padding(.vertical, 4)
        }
    }

    // MARK: - Helpers

    private var connColor: Color {
        switch bleManager.connectionState {
        case .connected: return .green; case .scanning, .connecting: return .yellow; case .disconnected: return .red
        }
    }
    private func zColor(_ z: PowerZone) -> Color {
        switch z { case .z1: return .gray; case .z2: return .blue; case .z3: return .green; case .z4: return .yellow; case .z5: return .orange; case .z6: return .red }
    }
    private var pwrColor: Color {
        let p = Int(bleManager.power)
        if p == 0 { return .gray }; if p < 100 { return .green }; if p < 200 { return .yellow }; if p < 300 { return .orange }; return .red
    }
    private var currentHeartRate: Double { connectivityManager.watchHeartRate > 0 ? connectivityManager.watchHeartRate : healthKitManager.heartRate }

    private func startRecording() {
        recordingCancellable = Timer.publish(every: 2, on: .main, in: .common).autoconnect()
            .sink { _ in
                workoutSession.recordPower(bleManager.power)
                workoutSession.recordCadence(bleManager.cadence)
                workoutSession.recordResistance(bleManager.resistance)
                workoutSession.recordHeartRate(currentHeartRate)
                workoutSession.recordSpeed(cadence: bleManager.cadence, resistance: bleManager.resistance)
                connectivityManager.sendMetrics(power: Int(bleManager.power), cadence: Int(bleManager.cadence), resistance: Int(bleManager.resistance))
            }
    }

    private func startWorkout() { workoutSession.ftp = ftp; workoutSession.start(); connectivityManager.sendWorkoutAction("start") }
    private func pauseWorkout() { workoutSession.pause(); connectivityManager.sendWorkoutAction("pause") }
    private func resumeWorkout() { workoutSession.resume(); connectivityManager.sendWorkoutAction("resume") }
    private func endWorkout() { connectivityManager.sendWorkoutAction("stop"); _ = workoutSession.stop(); showDetail = false; onWorkoutEnd() }

    private func fmtTime(_ t: TimeInterval) -> String {
        let h = Int(t)/3600; let m = (Int(t)%3600)/60; let s = Int(t)%60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
