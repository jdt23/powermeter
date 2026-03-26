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
            let controlHeight: CGFloat = 56
            let statusHeight: CGFloat = 34
            let metricsHeight = geo.size.height - controlHeight - statusHeight

            VStack(spacing: 0) {
                statusBar
                    .frame(height: statusHeight)

                metricsGrid(height: metricsHeight)

                workoutControls
                    .frame(height: controlHeight)
            }
        }
        .background(Color.black)
        .ignoresSafeArea(.container, edges: .bottom)
        .onAppear { startRecording() }
        .onDisappear { recordingCancellable?.cancel() }
        .confirmationDialog("End Workout?", isPresented: $showingStopConfirmation) {
            Button("End Workout", role: .destructive) { endWorkout() }
            Button("Cancel", role: .cancel) {}
        }
    }

    // MARK: - Status Bar

    private var statusBar: some View {
        HStack(spacing: 6) {
            Circle().fill(connectionColor).frame(width: 8, height: 8)
            Text(bleManager.connectionState.rawValue)
                .font(.system(size: 11)).foregroundColor(.gray)
            if connectivityManager.isWatchReachable {
                Image(systemName: "applewatch.radiowaves.left.and.right")
                    .font(.system(size: 10)).foregroundColor(.green)
            }
            Spacer()
            if workoutSession.state != .idle {
                Text(formatDuration(workoutSession.elapsed))
                    .font(.system(size: 24, weight: .bold, design: .monospaced))
                    .foregroundColor(.white)
            }
        }
        .padding(.horizontal, 14)
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
        let gap: CGFloat = 4
        let rowHeight = (height - gap * 2 - 8) / 3  // 3 rows, 2 gaps, small top/bottom pad

        return VStack(spacing: gap) {
            // Row 1: POWER
            powerCard(height: rowHeight, active: active)

            // Row 2: CADENCE | RESISTANCE
            HStack(spacing: gap) {
                cell(label: "CADENCE", value: "\(bleManager.cadence)", unit: "RPM",
                     color: .cyan, height: rowHeight,
                     avg: active ? "avg \(Int(workoutSession.averageCadence))" : nil,
                     peak: active && workoutSession.maxCadence > 0 ? "max \(Int(workoutSession.maxCadence))" : nil)
                cell(label: "RESISTANCE", value: "\(bleManager.resistance)", unit: "",
                     color: .orange, height: rowHeight,
                     avg: active ? "avg \(Int(workoutSession.averageResistance))" : nil,
                     peak: active && workoutSession.maxResistance > 0 ? "max \(Int(workoutSession.maxResistance))" : nil)
            }

            // Row 3: HR | CALORIES
            HStack(spacing: gap) {
                cell(label: "HEART RATE",
                     value: currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--",
                     unit: "BPM", color: .red, height: rowHeight,
                     avg: active && workoutSession.averageHeartRate > 0 ? "avg \(Int(workoutSession.averageHeartRate))" : nil,
                     peak: active && workoutSession.maxHeartRate > 0 ? "max \(Int(workoutSession.maxHeartRate))" : nil)
                cell(label: "CALORIES", value: "\(Int(workoutSession.totalCalories))", unit: "KCAL",
                     color: .yellow, height: rowHeight)
            }
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
    }

    // MARK: - Power Card

    private func powerCard(height: CGFloat, active: Bool) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 16)
                .fill(
                    LinearGradient(
                        colors: [powerColor.opacity(0.12), Color.white.opacity(0.02)],
                        startPoint: .top, endPoint: .bottom
                    )
                )
                .overlay(
                    RoundedRectangle(cornerRadius: 16)
                        .strokeBorder(powerColor.opacity(0.25), lineWidth: 1)
                )

            VStack(spacing: 0) {
                Text("POWER")
                    .font(.system(size: 11, weight: .bold))
                    .foregroundColor(powerColor.opacity(0.7))
                    .tracking(2)

                Spacer(minLength: 0)

                HStack(alignment: .lastTextBaseline, spacing: 4) {
                    Text("\(bleManager.power)")
                        .font(.system(size: min(height * 0.5, 110), weight: .heavy, design: .rounded))
                        .foregroundColor(.white)
                        .minimumScaleFactor(0.4)
                        .lineLimit(1)
                    Text("W")
                        .font(.system(size: 24, weight: .semibold))
                        .foregroundColor(Color.white.opacity(0.4))
                        .padding(.bottom, 6)
                }

                Spacer(minLength: 0)

                if active {
                    HStack(spacing: 20) {
                        statLabel("avg \(Int(workoutSession.averagePower))")
                        statLabel("max \(Int(workoutSession.maxPower))")
                    }
                    .foregroundColor(powerColor.opacity(0.45))
                    .padding(.bottom, 6)
                }
            }
            .padding(.vertical, 6)
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    // MARK: - Generic Metric Cell

    private func cell(
        label: String, value: String, unit: String,
        color: Color, height: CGFloat,
        avg: String? = nil, peak: String? = nil
    ) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 14)
                .fill(Color.white.opacity(0.035))
                .overlay(
                    RoundedRectangle(cornerRadius: 14)
                        .strokeBorder(color.opacity(0.15), lineWidth: 1)
                )

            VStack(spacing: 0) {
                Text(label)
                    .font(.system(size: 10, weight: .bold))
                    .foregroundColor(color.opacity(0.65))
                    .tracking(1.2)

                Spacer(minLength: 0)

                HStack(alignment: .lastTextBaseline, spacing: 2) {
                    Text(value)
                        .font(.system(size: min(height * 0.38, 56), weight: .bold, design: .rounded))
                        .foregroundColor(.white)
                        .minimumScaleFactor(0.4)
                        .lineLimit(1)
                    if !unit.isEmpty {
                        Text(unit)
                            .font(.system(size: 11, weight: .medium))
                            .foregroundColor(Color.white.opacity(0.35))
                            .padding(.bottom, 2)
                    }
                }

                Spacer(minLength: 0)

                if avg != nil || peak != nil {
                    HStack(spacing: 8) {
                        if let avg { statLabel(avg).foregroundColor(color.opacity(0.4)) }
                        if let peak { statLabel(peak).foregroundColor(color.opacity(0.4)) }
                    }
                    .padding(.bottom, 4)
                } else {
                    Spacer().frame(height: 18) // keep value vertically centered
                }
            }
            .padding(.vertical, 6)
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    private func statLabel(_ text: String) -> some View {
        Text(text)
            .font(.system(size: 12, weight: .semibold, design: .rounded))
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
        HStack(spacing: 10) {
            switch workoutSession.state {
            case .idle:
                Button(action: startWorkout) {
                    Label("Start Workout", systemImage: "play.fill")
                        .font(.system(size: 16, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(14)
                }
            case .active:
                Button(action: pauseWorkout) {
                    Image(systemName: "pause.fill")
                        .font(.system(size: 20, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.white.opacity(0.1))
                        .foregroundColor(.white)
                        .cornerRadius(14)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill")
                        .font(.system(size: 20, weight: .bold))
                        .frame(width: 70, maxHeight: .infinity)
                        .background(Color.red.opacity(0.8))
                        .foregroundColor(.white)
                        .cornerRadius(14)
                }
            case .paused:
                Button(action: resumeWorkout) {
                    Image(systemName: "play.fill")
                        .font(.system(size: 20, weight: .bold))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(14)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill")
                        .font(.system(size: 20, weight: .bold))
                        .frame(width: 70, maxHeight: .infinity)
                        .background(Color.red.opacity(0.8))
                        .foregroundColor(.white)
                        .cornerRadius(14)
                }
            }
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
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
