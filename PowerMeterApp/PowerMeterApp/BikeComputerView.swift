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

    private var hasSecondPage: Bool { workoutSession.state != .idle }

    // Use actual screen size — no safe area ambiguity
    private var screenW: CGFloat { UIScreen.main.bounds.width }
    private var screenH: CGFloat { UIScreen.main.bounds.height }

    private let ctrlH: CGFloat = 44
    private let barH: CGFloat = 24

    private var gridH: CGFloat { screenH - ctrlH - barH }

    var body: some View {
        VStack(spacing: 0) {
            statusBar.frame(width: screenW, height: barH)

            ZStack(alignment: .topLeading) {
                mainPage()
                    .frame(width: screenW, height: gridH)
                    .offset(x: page == 0 ? dragOffset : dragOffset - screenW)

                if hasSecondPage {
                    detailPage()
                        .frame(width: screenW, height: gridH)
                        .offset(x: page == 0 ? screenW + dragOffset : dragOffset)
                }
            }
            .frame(width: screenW, height: gridH)
            .clipped()
            .contentShape(Rectangle())
            .gesture(
                DragGesture()
                    .onChanged { v in
                        guard hasSecondPage else { return }
                        dragOffset = v.translation.width
                    }
                    .onEnded { v in
                        guard hasSecondPage else { return }
                        withAnimation(.easeOut(duration: 0.2)) {
                            if v.translation.width < -60 && page == 0 { page = 1 }
                            else if v.translation.width > 60 && page == 1 { page = 0 }
                            dragOffset = 0
                        }
                    }
            )

            workoutControls.frame(width: screenW, height: ctrlH)
        }
        .frame(width: screenW, height: screenH)
        .background(Color.black)
        .ignoresSafeArea()
        .onAppear { startRecording(); workoutSession.ftp = ftp }
        .onDisappear { recordingCancellable?.cancel() }
        .confirmationDialog("End Workout?", isPresented: $showingStopConfirmation) {
            Button("End Workout", role: .destructive) { endWorkout() }
            Button("Cancel", role: .cancel) {}
        }
        .sheet(isPresented: $showingSettings) { SettingsView() }
    }

    // MARK: - Status Bar

    private var statusBar: some View {
        HStack(spacing: 4) {
            Circle().fill(connectionColor).frame(width: 6, height: 6)
            Text(bleManager.connectionState.rawValue)
                .font(.system(size: 9)).foregroundColor(.gray)
            if connectivityManager.isWatchReachable {
                Image(systemName: "applewatch.radiowaves.left.and.right")
                    .font(.system(size: 8)).foregroundColor(.green)
            }
            Spacer()
            if hasSecondPage {
                HStack(spacing: 3) {
                    Circle().fill(page == 0 ? Color.white : Color.white.opacity(0.2))
                        .frame(width: 4, height: 4)
                    Circle().fill(page == 1 ? Color.white : Color.white.opacity(0.2))
                        .frame(width: 4, height: 4)
                }
                Spacer().frame(width: 6)
            }
            if workoutSession.state != .idle {
                Text(formatDuration(workoutSession.elapsed))
                    .font(.system(size: 18, weight: .bold, design: .monospaced))
                    .foregroundColor(.white)
            } else {
                Button(action: { showingSettings = true }) {
                    Image(systemName: "gearshape.fill")
                        .font(.system(size: 14)).foregroundColor(.gray)
                }
            }
        }
        .padding(.horizontal, 10)
    }

    private var connectionColor: Color {
        switch bleManager.connectionState {
        case .connected: return .green
        case .scanning, .connecting: return .yellow
        case .disconnected: return .red
        }
    }

    // MARK: - Page 1

    private func mainPage() -> some View {
        let active = workoutSession.state != .idle
        let g: CGFloat = 2
        let rh = (gridH - g * 3) / 4

        return VStack(spacing: g) {
            powerCard(height: rh, active: active)

            HStack(spacing: g) {
                cell("CADENCE", "\(bleManager.cadence)", "RPM", .cyan, rh,
                     avg: active ? "avg \(Int(workoutSession.averageCadence))" : nil,
                     peak: active && workoutSession.maxCadence > 0 ? "max \(Int(workoutSession.maxCadence))" : nil)
                cell("RESISTANCE", "\(bleManager.resistance)", "", .orange, rh,
                     avg: active ? "avg \(Int(workoutSession.averageResistance))" : nil,
                     peak: active && workoutSession.maxResistance > 0 ? "max \(Int(workoutSession.maxResistance))" : nil)
            }

            HStack(spacing: g) {
                cell("HEART RATE",
                     currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--",
                     "BPM", .red, rh,
                     avg: active && workoutSession.averageHeartRate > 0 ? "avg \(Int(workoutSession.averageHeartRate))" : nil,
                     peak: active && workoutSession.maxHeartRate > 0 ? "max \(Int(workoutSession.maxHeartRate))" : nil)
                cell("SPEED",
                     active ? String(format: "%.1f", workoutSession.speed) : "--",
                     "MPH", .mint, rh,
                     avg: active && workoutSession.averageSpeed > 0 ? String(format: "avg %.1f", workoutSession.averageSpeed) : nil,
                     peak: active && workoutSession.maxSpeed > 0 ? String(format: "max %.1f", workoutSession.maxSpeed) : nil)
            }

            HStack(spacing: g) {
                cell("DISTANCE",
                     active ? String(format: "%.2f", workoutSession.distance) : "--",
                     "MI", .mint, rh)
                cell("CALORIES", "\(Int(workoutSession.totalCalories))", "KCAL", .yellow, rh)
            }
        }
    }

    // MARK: - Page 2

    private func detailPage() -> some View {
        let g: CGFloat = 2
        let rh = (gridH - g) / 2

        return VStack(spacing: g) {
            HStack(spacing: g) {
                cell("NORMALIZED PWR", "\(Int(workoutSession.normalizedPower))", "W", .purple, rh)
                cell("INTENSITY", String(format: "%.2f", workoutSession.intensityFactor), "IF", .indigo, rh)
            }

            HStack(spacing: g) {
                cell("TRAINING STRESS", "\(Int(workoutSession.tss))", "TSS", .pink, rh)
                zoneCell(height: rh)
            }
        }
    }

    // MARK: - Zone Cell

    private func zoneCell(height: CGFloat) -> some View {
        let zone = workoutSession.powerZone
        let color = zoneSwiftColor(zone)

        return ZStack {
            RoundedRectangle(cornerRadius: 8)
                .fill(color.opacity(0.1))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(color.opacity(0.3), lineWidth: 1))

            VStack(spacing: 0) {
                Text("ZONE").font(.system(size: 9, weight: .bold))
                    .foregroundColor(color.opacity(0.65)).tracking(1)
                Spacer(minLength: 0)
                Text("Z\(zone.rawValue)")
                    .font(.system(size: min(height * 0.35, 80), weight: .heavy, design: .rounded))
                    .foregroundColor(color)
                Text(zone.label)
                    .font(.system(size: 16, weight: .bold))
                    .foregroundColor(color.opacity(0.7))
                Spacer(minLength: 0)
            }
            .padding(.top, 2)
        }
        .frame(maxWidth: .infinity).frame(height: height)
    }

    // MARK: - Power Card

    private func powerCard(height: CGFloat, active: Bool) -> some View {
        let zone = workoutSession.powerZone
        let zc = active ? zoneSwiftColor(zone) : powerColor

        return ZStack {
            RoundedRectangle(cornerRadius: 8)
                .fill(LinearGradient(colors: [zc.opacity(0.15), Color.white.opacity(0.02)],
                                     startPoint: .top, endPoint: .bottom))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(zc.opacity(0.3), lineWidth: 1))

            VStack(spacing: 0) {
                HStack {
                    Text("POWER").font(.system(size: 9, weight: .bold))
                        .foregroundColor(zc.opacity(0.7)).tracking(2)
                    Spacer()
                    if active {
                        Text("Z\(zone.rawValue) \(zone.label)")
                            .font(.system(size: 9, weight: .bold))
                            .foregroundColor(zc.opacity(0.8))
                            .padding(.horizontal, 6).padding(.vertical, 1)
                            .background(Capsule().fill(zc.opacity(0.15)))
                    }
                }
                .padding(.horizontal, 8)

                Spacer(minLength: 0)

                HStack(alignment: .lastTextBaseline, spacing: 3) {
                    Text("\(bleManager.power)")
                        .font(.system(size: min(height * 0.52, 100), weight: .heavy, design: .rounded))
                        .foregroundColor(.white).minimumScaleFactor(0.4).lineLimit(1)
                    Text("W").font(.system(size: 18, weight: .semibold))
                        .foregroundColor(Color.white.opacity(0.4)).padding(.bottom, 2)
                }

                Spacer(minLength: 0)

                if active {
                    HStack(spacing: 14) {
                        statLabel("avg \(Int(workoutSession.averagePower))")
                        statLabel("max \(Int(workoutSession.maxPower))")
                    }
                    .foregroundColor(zc.opacity(0.45)).padding(.bottom, 2)
                }
            }
            .padding(.top, 2)
        }
        .frame(maxWidth: .infinity).frame(height: height)
    }

    // MARK: - Cell

    private func cell(
        _ label: String, _ value: String, _ unit: String,
        _ color: Color, _ height: CGFloat,
        avg: String? = nil, peak: String? = nil
    ) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 8)
                .fill(Color.white.opacity(0.035))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(color.opacity(0.15), lineWidth: 1))

            VStack(spacing: 0) {
                Text(label).font(.system(size: 9, weight: .bold))
                    .foregroundColor(color.opacity(0.65)).tracking(1)
                    .lineLimit(1).minimumScaleFactor(0.7)

                Spacer(minLength: 0)

                HStack(alignment: .lastTextBaseline, spacing: 2) {
                    Text(value)
                        .font(.system(size: min(height * 0.36, 48), weight: .bold, design: .rounded))
                        .foregroundColor(.white).minimumScaleFactor(0.4).lineLimit(1)
                    if !unit.isEmpty {
                        Text(unit).font(.system(size: 9, weight: .medium))
                            .foregroundColor(Color.white.opacity(0.3)).padding(.bottom, 1)
                    }
                }

                Spacer(minLength: 0)

                if avg != nil || peak != nil {
                    HStack(spacing: 4) {
                        if let avg { Text(avg).foregroundColor(color.opacity(0.4)) }
                        if let peak { Text(peak).foregroundColor(color.opacity(0.4)) }
                    }
                    .font(.system(size: 9, weight: .semibold, design: .rounded))
                    .lineLimit(1).minimumScaleFactor(0.7)
                    .padding(.bottom, 1)
                }
            }
            .padding(.top, 2)
        }
        .frame(maxWidth: .infinity).frame(height: height)
    }

    private func statLabel(_ text: String) -> some View {
        Text(text).font(.system(size: 10, weight: .semibold, design: .rounded))
    }

    private func zoneSwiftColor(_ zone: PowerZone) -> Color {
        switch zone {
        case .z1: return .gray; case .z2: return .blue; case .z3: return .green
        case .z4: return .yellow; case .z5: return .orange; case .z6: return .red
        }
    }

    private var powerColor: Color {
        let p = Int(bleManager.power)
        if p == 0 { return .gray }
        if p < 100 { return .green }
        if p < 200 { return .yellow }
        if p < 300 { return .orange }
        return .red
    }

    // MARK: - Controls

    private var workoutControls: some View {
        HStack(spacing: 6) {
            switch workoutSession.state {
            case .idle:
                Button(action: startWorkout) {
                    Label("Start Workout", systemImage: "play.fill")
                        .font(.system(size: 14, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.green).foregroundColor(.black).cornerRadius(8)
                }
            case .active:
                Button(action: pauseWorkout) {
                    Image(systemName: "pause.fill").font(.system(size: 16, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.white.opacity(0.1)).foregroundColor(.white).cornerRadius(8)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill").font(.system(size: 16, weight: .bold))
                        .frame(maxHeight: .infinity).frame(width: 54)
                        .background(Color.red.opacity(0.8)).foregroundColor(.white).cornerRadius(8)
                }
            case .paused:
                Button(action: resumeWorkout) {
                    Image(systemName: "play.fill").font(.system(size: 16, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.green).foregroundColor(.black).cornerRadius(8)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill").font(.system(size: 16, weight: .bold))
                        .frame(maxHeight: .infinity).frame(width: 54)
                        .background(Color.red.opacity(0.8)).foregroundColor(.white).cornerRadius(8)
                }
            }
        }
        .padding(.horizontal, 2).padding(.vertical, 1)
    }

    private var currentHeartRate: Double {
        connectivityManager.watchHeartRate > 0 ? connectivityManager.watchHeartRate : healthKitManager.heartRate
    }

    // MARK: - Recording

    private func startRecording() {
        recordingCancellable = Timer.publish(every: 2, on: .main, in: .common)
            .autoconnect()
            .sink { _ in
                workoutSession.recordPower(bleManager.power)
                workoutSession.recordCadence(bleManager.cadence)
                workoutSession.recordResistance(bleManager.resistance)
                workoutSession.recordHeartRate(currentHeartRate)
                workoutSession.recordSpeed(cadence: bleManager.cadence, resistance: bleManager.resistance)
                connectivityManager.sendMetrics(
                    power: Int(bleManager.power),
                    cadence: Int(bleManager.cadence),
                    resistance: Int(bleManager.resistance)
                )
            }
    }

    private func startWorkout() {
        workoutSession.ftp = ftp
        workoutSession.start()
        connectivityManager.sendWorkoutAction("start")
    }
    private func pauseWorkout() {
        workoutSession.pause()
        connectivityManager.sendWorkoutAction("pause")
    }
    private func resumeWorkout() {
        workoutSession.resume()
        connectivityManager.sendWorkoutAction("resume")
    }
    private func endWorkout() {
        connectivityManager.sendWorkoutAction("stop")
        _ = workoutSession.stop()
        page = 0
        onWorkoutEnd()
    }

    private func formatDuration(_ interval: TimeInterval) -> String {
        let h = Int(interval) / 3600
        let m = (Int(interval) % 3600) / 60
        let s = Int(interval) % 60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
