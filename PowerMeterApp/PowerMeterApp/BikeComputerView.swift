import SwiftUI
import Combine

struct BikeComputerView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @EnvironmentObject var connectivityManager: PhoneConnectivityManager

    var onWorkoutEnd: () -> Void

    @State private var showingStopConfirmation = false
    @State private var recordingCancellable: AnyCancellable?

    var body: some View {
        GeometryReader { geo in
            let total = geo.size.height + geo.safeAreaInsets.top + geo.safeAreaInsets.bottom
            let controlH: CGFloat = 50
            let statusH: CGFloat = 30
            let metricsH = total - controlH - statusH

            VStack(spacing: 0) {
                statusBar.frame(height: statusH)
                metricsGrid(height: metricsH)
                workoutControls.frame(height: controlH)
            }
            .frame(width: geo.size.width, height: total)
        }
        .ignoresSafeArea()
        .background(Color.black)
        .onAppear { startRecording() }
        .onDisappear { recordingCancellable?.cancel() }
        .confirmationDialog("End Workout?", isPresented: $showingStopConfirmation) {
            Button("End Workout", role: .destructive) { endWorkout() }
            Button("Cancel", role: .cancel) {}
        }
    }

    // MARK: - Status Bar

    private var statusBar: some View {
        HStack(spacing: 5) {
            Circle().fill(connectionColor).frame(width: 7, height: 7)
            Text(bleManager.connectionState.rawValue)
                .font(.system(size: 10)).foregroundColor(.gray)
            if connectivityManager.isWatchReachable {
                Image(systemName: "applewatch.radiowaves.left.and.right")
                    .font(.system(size: 9)).foregroundColor(.green)
            }
            Spacer()
            if workoutSession.state != .idle {
                Text(formatDuration(workoutSession.elapsed))
                    .font(.system(size: 20, weight: .bold, design: .monospaced))
                    .foregroundColor(.white)
            }
        }
        .padding(.horizontal, 12)
    }

    private var connectionColor: Color {
        switch bleManager.connectionState {
        case .connected: return .green
        case .scanning, .connecting: return .yellow
        case .disconnected: return .red
        }
    }

    // MARK: - Metrics

    private func metricsGrid(height: CGFloat) -> some View {
        let active = workoutSession.state != .idle
        let gap: CGFloat = 3
        let rows: CGFloat = 4
        let rowH = (height - gap * (rows - 1)) / rows

        return VStack(spacing: gap) {
            // Row 1: POWER with zone badge
            powerCard(height: rowH, active: active)

            // Row 2: CADENCE | SPEED
            HStack(spacing: gap) {
                cell("CADENCE", "\(bleManager.cadence)", "RPM", .cyan, rowH,
                     avg: active ? "avg \(Int(workoutSession.averageCadence))" : nil,
                     peak: active && workoutSession.maxCadence > 0 ? "max \(Int(workoutSession.maxCadence))" : nil)
                cell("SPEED", active ? String(format: "%.1f", workoutSession.speed) : "--", "MPH", .mint, rowH,
                     avg: active && workoutSession.averageSpeed > 0 ? String(format: "avg %.1f", workoutSession.averageSpeed) : nil,
                     peak: active && workoutSession.maxSpeed > 0 ? String(format: "max %.1f", workoutSession.maxSpeed) : nil)
            }

            // Row 3: HEART RATE | RESISTANCE
            HStack(spacing: gap) {
                cell("HEART RATE",
                     currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--",
                     "BPM", .red, rowH,
                     avg: active && workoutSession.averageHeartRate > 0 ? "avg \(Int(workoutSession.averageHeartRate))" : nil,
                     peak: active && workoutSession.maxHeartRate > 0 ? "max \(Int(workoutSession.maxHeartRate))" : nil)
                cell("RESISTANCE", "\(bleManager.resistance)", "", .orange, rowH,
                     avg: active ? "avg \(Int(workoutSession.averageResistance))" : nil,
                     peak: active && workoutSession.maxResistance > 0 ? "max \(Int(workoutSession.maxResistance))" : nil)
            }

            // Row 4: DISTANCE | NP | CALORIES  (or DISTANCE | IF | TSS when workout active)
            if active {
                HStack(spacing: gap) {
                    miniCell("DISTANCE", String(format: "%.2f", workoutSession.distance), "MI", .mint, rowH)
                    miniCell("NP", "\(Int(workoutSession.normalizedPower))", "W", .purple, rowH)
                    miniCell("IF", String(format: "%.2f", workoutSession.intensityFactor), "", .indigo, rowH)
                    miniCell("TSS", "\(Int(workoutSession.tss))", "", .pink, rowH)
                    miniCell("KCAL", "\(Int(workoutSession.totalCalories))", "", .yellow, rowH)
                }
            } else {
                HStack(spacing: gap) {
                    cell("DISTANCE", "--", "MI", .mint, rowH)
                    cell("CALORIES", "0", "KCAL", .yellow, rowH)
                }
            }
        }
        .padding(.horizontal, 3)
    }

    // MARK: - Power Card

    private func powerCard(height: CGFloat, active: Bool) -> some View {
        let zone = workoutSession.powerZone
        let zoneColor = active ? zoneSwiftColor(zone) : powerColor

        return ZStack {
            RoundedRectangle(cornerRadius: 12)
                .fill(
                    LinearGradient(
                        colors: [zoneColor.opacity(0.15), Color.white.opacity(0.02)],
                        startPoint: .top, endPoint: .bottom
                    )
                )
                .overlay(
                    RoundedRectangle(cornerRadius: 12)
                        .strokeBorder(zoneColor.opacity(0.3), lineWidth: 1)
                )

            VStack(spacing: 0) {
                HStack {
                    Text("POWER")
                        .font(.system(size: 10, weight: .bold))
                        .foregroundColor(zoneColor.opacity(0.7))
                        .tracking(2)
                    Spacer()
                    if active {
                        Text("Z\(zone.rawValue) \(zone.label)")
                            .font(.system(size: 10, weight: .bold))
                            .foregroundColor(zoneColor.opacity(0.8))
                            .padding(.horizontal, 8)
                            .padding(.vertical, 2)
                            .background(Capsule().fill(zoneColor.opacity(0.15)))
                    }
                }
                .padding(.horizontal, 12)

                Spacer(minLength: 0)

                HStack(alignment: .lastTextBaseline, spacing: 4) {
                    Text("\(bleManager.power)")
                        .font(.system(size: min(height * 0.5, 100), weight: .heavy, design: .rounded))
                        .foregroundColor(.white)
                        .minimumScaleFactor(0.4)
                        .lineLimit(1)
                    Text("W")
                        .font(.system(size: 22, weight: .semibold))
                        .foregroundColor(Color.white.opacity(0.4))
                        .padding(.bottom, 4)
                }

                Spacer(minLength: 0)

                if active {
                    HStack(spacing: 16) {
                        statLabel("avg \(Int(workoutSession.averagePower))")
                        statLabel("max \(Int(workoutSession.maxPower))")
                        if workoutSession.normalizedPower > 0 {
                            statLabel("NP \(Int(workoutSession.normalizedPower))")
                        }
                    }
                    .foregroundColor(zoneColor.opacity(0.45))
                    .padding(.bottom, 3)
                }
            }
            .padding(.top, 4)
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    private func zoneSwiftColor(_ zone: PowerZone) -> Color {
        switch zone {
        case .z1: return .gray
        case .z2: return .blue
        case .z3: return .green
        case .z4: return .yellow
        case .z5: return .orange
        case .z6: return .red
        }
    }

    // MARK: - Standard Cell

    private func cell(
        _ label: String, _ value: String, _ unit: String,
        _ color: Color, _ height: CGFloat,
        avg: String? = nil, peak: String? = nil
    ) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 10)
                .fill(Color.white.opacity(0.035))
                .overlay(
                    RoundedRectangle(cornerRadius: 10)
                        .strokeBorder(color.opacity(0.15), lineWidth: 1)
                )

            VStack(spacing: 0) {
                Text(label)
                    .font(.system(size: 9, weight: .bold))
                    .foregroundColor(color.opacity(0.65))
                    .tracking(1)

                Spacer(minLength: 0)

                HStack(alignment: .lastTextBaseline, spacing: 2) {
                    Text(value)
                        .font(.system(size: min(height * 0.38, 50), weight: .bold, design: .rounded))
                        .foregroundColor(.white)
                        .minimumScaleFactor(0.4)
                        .lineLimit(1)
                    if !unit.isEmpty {
                        Text(unit)
                            .font(.system(size: 10, weight: .medium))
                            .foregroundColor(Color.white.opacity(0.3))
                            .padding(.bottom, 1)
                    }
                }

                Spacer(minLength: 0)

                if avg != nil || peak != nil {
                    HStack(spacing: 5) {
                        if let avg { Text(avg).foregroundColor(color.opacity(0.4)) }
                        if let peak { Text(peak).foregroundColor(color.opacity(0.4)) }
                    }
                    .font(.system(size: 10, weight: .semibold, design: .rounded))
                    .padding(.bottom, 2)
                }
            }
            .padding(.top, 3)
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    // MARK: - Mini Cell (for bottom row with 5 items)

    private func miniCell(
        _ label: String, _ value: String, _ unit: String,
        _ color: Color, _ height: CGFloat
    ) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 8)
                .fill(Color.white.opacity(0.035))
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .strokeBorder(color.opacity(0.12), lineWidth: 1)
                )

            VStack(spacing: 1) {
                Text(label)
                    .font(.system(size: 8, weight: .bold))
                    .foregroundColor(color.opacity(0.6))
                    .tracking(0.8)

                Spacer(minLength: 0)

                Text(value)
                    .font(.system(size: min(height * 0.28, 26), weight: .bold, design: .rounded))
                    .foregroundColor(.white)
                    .minimumScaleFactor(0.5)
                    .lineLimit(1)

                if !unit.isEmpty {
                    Text(unit)
                        .font(.system(size: 8, weight: .medium))
                        .foregroundColor(Color.white.opacity(0.3))
                }

                Spacer(minLength: 0)
            }
            .padding(.top, 2)
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    private func statLabel(_ text: String) -> some View {
        Text(text).font(.system(size: 11, weight: .semibold, design: .rounded))
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
        HStack(spacing: 8) {
            switch workoutSession.state {
            case .idle:
                Button(action: startWorkout) {
                    Label("Start Workout", systemImage: "play.fill")
                        .font(.system(size: 15, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(10)
                }
            case .active:
                Button(action: pauseWorkout) {
                    Image(systemName: "pause.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.white.opacity(0.1))
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(maxHeight: .infinity)
                        .frame(width: 60)
                        .background(Color.red.opacity(0.8))
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
            case .paused:
                Button(action: resumeWorkout) {
                    Image(systemName: "play.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(10)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(maxHeight: .infinity)
                        .frame(width: 60)
                        .background(Color.red.opacity(0.8))
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
            }
        }
        .padding(.horizontal, 3)
        .padding(.vertical, 2)
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
        onWorkoutEnd()
    }

    private func formatDuration(_ interval: TimeInterval) -> String {
        let h = Int(interval) / 3600
        let m = (Int(interval) % 3600) / 60
        let s = Int(interval) % 60
        return h > 0 ? String(format: "%d:%02d:%02d", h, m, s) : String(format: "%02d:%02d", m, s)
    }
}
