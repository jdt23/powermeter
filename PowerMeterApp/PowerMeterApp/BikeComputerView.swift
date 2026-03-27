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
    @State private var page = 0
    @State private var dragOffset: CGFloat = 0
    @AppStorage("ftp") private var ftp: Double = 200.0
    @AppStorage("maxHR") private var maxHR: Double = 190.0

    private var on: Bool { workoutSession.state != .idle }
    private var hasPage2: Bool { on }
    private var hr: Double { connectivityManager.watchHeartRate > 0 ? connectivityManager.watchHeartRate : healthKitManager.heartRate }

    var body: some View {
        VStack(spacing: 2) {
            // Status bar — fixed height
            HStack(spacing: 4) {
                Circle().fill(connColor).frame(width: 6, height: 6)
                Text(bleManager.connectionState.rawValue).font(.system(size: 9)).foregroundColor(.gray)
                if connectivityManager.isWatchReachable {
                    Image(systemName: "applewatch.radiowaves.left.and.right").font(.system(size: 8)).foregroundColor(.green)
                }
                Spacer()
                if hasPage2 {
                    HStack(spacing: 3) {
                        Circle().fill(page == 0 ? Color.white : Color.white.opacity(0.2)).frame(width: 4, height: 4)
                        Circle().fill(page == 1 ? Color.white : Color.white.opacity(0.2)).frame(width: 4, height: 4)
                    }
                    Spacer().frame(width: 6)
                }
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
            .frame(height: 28)

            // Metrics — swipeable, fills ALL remaining space
            GeometryReader { geo in
                let w = geo.size.width
                let h = geo.size.height
                HStack(spacing: 0) {
                    mainPage(h: h).frame(width: w, height: h)
                    if hasPage2 {
                        detailPage(h: h).frame(width: w, height: h)
                    }
                }
                .frame(height: h)
                .offset(x: page == 0 ? dragOffset : -w + dragOffset)
                .gesture(
                    DragGesture()
                        .onChanged { v in guard hasPage2 else { return }; dragOffset = v.translation.width }
                        .onEnded { v in
                            guard hasPage2 else { return }
                            withAnimation(.easeOut(duration: 0.2)) {
                                if v.translation.width < -60 && page == 0 { page = 1 }
                                else if v.translation.width > 60 && page == 1 { page = 0 }
                                dragOffset = 0
                            }
                        }
                )
            }
            .clipped()

            // Controls — fixed height
            HStack(spacing: 6) {
                if !on {
                    Button(action: startWorkout) {
                        Label("Start Workout", systemImage: "play.fill").font(.system(size: 14, weight: .bold))
                            .frame(maxWidth: .infinity, maxHeight: .infinity).background(Color.green).foregroundColor(.black).cornerRadius(8)
                    }
                } else {
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
                    Button(action: { showingStopConfirmation = true }) {
                        Image(systemName: "stop.fill").font(.system(size: 16, weight: .bold))
                            .frame(maxHeight: .infinity).frame(width: 54).background(Color.red.opacity(0.8)).foregroundColor(.white).cornerRadius(8)
                    }
                }
            }
            .padding(.horizontal, 2)
            .frame(height: 44)
        }
        .background(Color.black)
        .onAppear { startRecording(); workoutSession.ftp = ftp; workoutSession.maxHR = maxHR }
        .onDisappear { recordingCancellable?.cancel() }
        .confirmationDialog("End Workout?", isPresented: $showingStopConfirmation) {
            Button("End Workout", role: .destructive) { endWorkout() }
            Button("Cancel", role: .cancel) {}
        }
        .sheet(isPresented: $showingSettings) { SettingsView() }
    }

    // MARK: - Page 1
    // Each row gets an explicit height = h / numberOfRows

    private func mainPage(h: CGFloat) -> some View {
        let rows: CGFloat = 4
        let gaps: CGFloat = 2 * (rows - 1)
        let rowH = (h - gaps) / rows
        let z = workoutSession.hrZone
        let zc = hr > 0 ? zoneColor(z) : Color.gray

        return VStack(spacing: 2) {
            // Row 1: HR hero
            ZStack {
                RoundedRectangle(cornerRadius: 8)
                    .fill(LinearGradient(colors: [zc.opacity(0.15), Color.white.opacity(0.02)], startPoint: .top, endPoint: .bottom))
                    .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(zc.opacity(0.3), lineWidth: 1))
                VStack {
                    HStack {
                        Text("HEART RATE").font(.system(size: 9, weight: .bold)).foregroundColor(zc.opacity(0.7)).tracking(2)
                        Spacer()
                        if on && hr > 0 {
                            Text("Z\(z.rawValue) \(z.label)").font(.system(size: 9, weight: .bold)).foregroundColor(zc.opacity(0.8))
                                .padding(.horizontal, 6).padding(.vertical, 1).background(Capsule().fill(zc.opacity(0.15)))
                        }
                    }.padding(.horizontal, 8)
                    Spacer(minLength: 0)
                    HStack(alignment: .lastTextBaseline, spacing: 3) {
                        Text(hr > 0 ? "\(Int(hr))" : "--")
                            .font(.system(size: 72, weight: .heavy, design: .rounded))
                            .foregroundColor(.white).minimumScaleFactor(0.3).lineLimit(1)
                        Text("BPM").font(.system(size: 16, weight: .semibold)).foregroundColor(Color.white.opacity(0.4))
                    }
                    Spacer(minLength: 0)
                    if on && workoutSession.averageHeartRate > 0 {
                        HStack(spacing: 14) {
                            Text("avg \(Int(workoutSession.averageHeartRate))").font(.system(size: 13, weight: .semibold, design: .rounded))
                            Text("max \(Int(workoutSession.maxHeartRate))").font(.system(size: 13, weight: .semibold, design: .rounded))
                        }.foregroundColor(zc.opacity(0.45)).padding(.bottom, 2)
                    }
                }.padding(.top, 4)
            }
            .frame(height: rowH)

            // Row 2: Cadence | Power
            HStack(spacing: 2) {
                tile("CADENCE", "\(bleManager.cadence)", "RPM", .cyan,
                     on ? "avg \(Int(workoutSession.averageCadence))" : nil,
                     on && workoutSession.maxCadence > 0 ? "max \(Int(workoutSession.maxCadence))" : nil)
                tile("POWER", "\(bleManager.power)", "W", .green,
                     on ? "avg \(Int(workoutSession.averagePower))" : nil,
                     on && workoutSession.maxPower > 0 ? "max \(Int(workoutSession.maxPower))" : nil)
            }
            .frame(height: rowH)

            // Row 3: Speed | Distance
            HStack(spacing: 2) {
                tile("SPEED", on || workoutSession.speed > 0 ? String(format: "%.1f", workoutSession.speed) : "--", "MPH", .mint,
                     on && workoutSession.averageSpeed > 0 ? String(format: "avg %.1f", workoutSession.averageSpeed) : nil,
                     on && workoutSession.maxSpeed > 0 ? String(format: "max %.1f", workoutSession.maxSpeed) : nil)
                tile("DISTANCE", on ? String(format: "%.2f", workoutSession.distance) : "--", "MI", .mint, nil, nil)
            }
            .frame(height: rowH)

            // Row 4: Resistance | Calories
            HStack(spacing: 2) {
                tile("RESISTANCE", "\(bleManager.resistance)", "", .orange,
                     on ? "avg \(Int(workoutSession.averageResistance))" : nil,
                     on && workoutSession.maxResistance > 0 ? "max \(Int(workoutSession.maxResistance))" : nil)
                tile("CALORIES", "\(Int(workoutSession.totalCalories))", "KCAL", .yellow, nil, nil)
            }
            .frame(height: rowH)
        }
    }

    // MARK: - Page 2

    private func detailPage(h: CGFloat) -> some View {
        let rowH = (h - 2) / 2

        return VStack(spacing: 2) {
            HStack(spacing: 2) {
                tile("NORMALIZED PWR", "\(Int(workoutSession.normalizedPower))", "W", .purple, nil, nil)
                tile("INTENSITY", String(format: "%.2f", workoutSession.intensityFactor), "IF", .indigo, nil, nil)
            }
            .frame(height: rowH)

            HStack(spacing: 2) {
                tile("TRAINING STRESS", "\(Int(workoutSession.tss))", "TSS", .pink, nil, nil)
                tile("FTP", "\(Int(ftp))", "W", .gray, nil, nil)
            }
            .frame(height: rowH)
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
                    HStack(spacing: 6) {
                        if let a = avg { Text(a).foregroundColor(color.opacity(0.5)) }
                        if let p = peak { Text(p).foregroundColor(color.opacity(0.5)) }
                    }.font(.system(size: 13, weight: .semibold, design: .rounded)).lineLimit(1).minimumScaleFactor(0.7)
                }
            }.padding(.vertical, 4)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    // MARK: - Helpers

    private var connColor: Color {
        switch bleManager.connectionState {
        case .connected: return .green; case .scanning, .connecting: return .yellow; case .disconnected: return .red
        }
    }

    private func zoneColor(_ z: HeartRateZone) -> Color {
        switch z { case .z1: return .gray; case .z2: return .blue; case .z3: return .green; case .z4: return .orange; case .z5: return .red }
    }

    private func startRecording() {
        recordingCancellable = Timer.publish(every: 2, on: .main, in: .common).autoconnect()
            .sink { _ in
                if on {
                    workoutSession.recordPower(bleManager.power)
                    workoutSession.recordCadence(bleManager.cadence)
                    workoutSession.recordResistance(bleManager.resistance)
                    workoutSession.recordHeartRate(hr)
                    workoutSession.recordSpeed(cadence: bleManager.cadence, resistance: bleManager.resistance)
                }
                connectivityManager.sendMetrics(
                    power: Int(bleManager.power), cadence: Int(bleManager.cadence), resistance: Int(bleManager.resistance),
                    speed: workoutSession.speed, distance: workoutSession.distance, calories: workoutSession.totalCalories)
            }
    }

    private func startWorkout() {
        workoutSession.ftp = ftp; workoutSession.maxHR = maxHR
        workoutSession.start()
        connectivityManager.sendWorkoutAction("start")
    }
    private func pauseWorkout() { workoutSession.pause(); connectivityManager.sendWorkoutAction("pause") }
    private func resumeWorkout() { workoutSession.resume(); connectivityManager.sendWorkoutAction("resume") }
    private func endWorkout() {
        connectivityManager.sendWorkoutAction("stop")
        workoutSession.watchSavedWorkout = connectivityManager.watchSavedWorkout
        _ = workoutSession.stop()
        page = 0
        onWorkoutEnd()
    }

    private func fmtTime(_ t: TimeInterval) -> String {
        let h = Int(t)/3600; let m = (Int(t)%3600)/60; let s = Int(t)%60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
