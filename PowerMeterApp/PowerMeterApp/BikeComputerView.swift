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
            VStack(spacing: 0) {
                statusBar
                metricsGrid(height: geo.size.height - 110)
                Spacer(minLength: 0)
                workoutControls
                    .padding(.bottom, geo.safeAreaInsets.bottom > 0 ? 4 : 12)
            }
        }
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
        HStack(spacing: 6) {
            Circle()
                .fill(connectionColor)
                .frame(width: 8, height: 8)
            Text(bleManager.connectionState.rawValue)
                .font(.system(size: 11))
                .foregroundColor(.gray)
            if connectivityManager.isWatchReachable {
                Image(systemName: "applewatch.radiowaves.left.and.right")
                    .font(.system(size: 10))
                    .foregroundColor(.green)
            }
            Spacer()
            if workoutSession.state != .idle {
                Text(formatDuration(workoutSession.elapsed))
                    .font(.system(size: 22, weight: .bold, design: .monospaced))
                    .foregroundColor(.white)
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 4)
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
        let isWorkout = workoutSession.state != .idle
        let rows: CGFloat = 3
        let spacing: CGFloat = 5
        let rowHeight = (height - spacing * (rows - 1) - 16) / rows

        return VStack(spacing: spacing) {
            // Row 1: POWER (hero)
            powerCard(height: rowHeight, isWorkout: isWorkout)

            // Row 2: CADENCE | RESISTANCE
            HStack(spacing: spacing) {
                metricCell(
                    label: "CADENCE", value: "\(bleManager.cadence)", unit: "RPM",
                    color: .cyan, height: rowHeight,
                    avg: isWorkout ? "avg \(Int(workoutSession.averageCadence))" : nil
                )
                metricCell(
                    label: "RESISTANCE", value: "\(bleManager.resistance)", unit: "",
                    color: .orange, height: rowHeight
                )
            }

            // Row 3: HR | CALORIES
            HStack(spacing: spacing) {
                metricCell(
                    label: "HEART RATE",
                    value: currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--",
                    unit: "BPM", color: .red, height: rowHeight,
                    avg: isWorkout && workoutSession.averageHeartRate > 0 ? "avg \(Int(workoutSession.averageHeartRate))" : nil,
                    secondary: isWorkout && workoutSession.maxHeartRate > 0 ? "max \(Int(workoutSession.maxHeartRate))" : nil
                )
                metricCell(
                    label: "CALORIES", value: "\(Int(workoutSession.totalCalories))", unit: "KCAL",
                    color: .yellow, height: rowHeight
                )
            }
        }
        .padding(.horizontal, 10)
        .padding(.top, 4)
    }

    private func powerCard(height: CGFloat, isWorkout: Bool) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 16)
                .fill(
                    LinearGradient(
                        colors: [powerColor.opacity(0.15), Color.white.opacity(0.03)],
                        startPoint: .top, endPoint: .bottom
                    )
                )
                .overlay(
                    RoundedRectangle(cornerRadius: 16)
                        .strokeBorder(powerColor.opacity(0.3), lineWidth: 1)
                )

            VStack(spacing: 0) {
                Text("POWER")
                    .font(.system(size: 11, weight: .bold))
                    .foregroundColor(powerColor.opacity(0.8))
                    .tracking(2)

                HStack(alignment: .lastTextBaseline, spacing: 4) {
                    Text("\(bleManager.power)")
                        .font(.system(size: min(height * 0.55, 96), weight: .heavy, design: .rounded))
                        .foregroundColor(.white)
                        .minimumScaleFactor(0.5)
                        .lineLimit(1)
                    Text("W")
                        .font(.system(size: 22, weight: .semibold))
                        .foregroundColor(.gray)
                        .padding(.bottom, 4)
                }

                if isWorkout {
                    HStack(spacing: 16) {
                        Text("avg \(Int(workoutSession.averagePower)) W")
                        Text("max \(Int(workoutSession.maxPower)) W")
                    }
                    .font(.system(size: 14, weight: .semibold, design: .rounded))
                    .foregroundColor(powerColor.opacity(0.5))
                }
            }
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    private func metricCell(
        label: String, value: String, unit: String,
        color: Color, height: CGFloat,
        avg: String? = nil, secondary: String? = nil
    ) -> some View {
        ZStack {
            RoundedRectangle(cornerRadius: 14)
                .fill(Color.white.opacity(0.04))
                .overlay(
                    RoundedRectangle(cornerRadius: 14)
                        .strokeBorder(color.opacity(0.15), lineWidth: 1)
                )

            VStack(spacing: 0) {
                Text(label)
                    .font(.system(size: 10, weight: .bold))
                    .foregroundColor(color.opacity(0.7))
                    .tracking(1.5)

                HStack(alignment: .lastTextBaseline, spacing: 2) {
                    Text(value)
                        .font(.system(size: min(height * 0.4, 52), weight: .bold, design: .rounded))
                        .foregroundColor(.white)
                        .minimumScaleFactor(0.5)
                        .lineLimit(1)
                    if !unit.isEmpty {
                        Text(unit)
                            .font(.system(size: 12, weight: .medium))
                            .foregroundColor(.gray)
                            .padding(.bottom, 2)
                    }
                }

                if avg != nil || secondary != nil {
                    HStack(spacing: 8) {
                        if let avg {
                            Text(avg)
                                .font(.system(size: 11, weight: .medium, design: .rounded))
                                .foregroundColor(color.opacity(0.45))
                        }
                        if let secondary {
                            Text(secondary)
                                .font(.system(size: 11, weight: .medium, design: .rounded))
                                .foregroundColor(color.opacity(0.45))
                        }
                    }
                }
            }
        }
        .frame(maxWidth: .infinity)
        .frame(height: height)
    }

    private var powerColor: Color {
        let p = Int(bleManager.power)
        if p == 0 { return .gray }
        if p < 100 { return .green }
        if p < 200 { return .yellow }
        if p < 300 { return .orange }
        return .red
    }

    // MARK: - Workout Controls

    private var workoutControls: some View {
        HStack(spacing: 12) {
            switch workoutSession.state {
            case .idle:
                Button(action: startWorkout) {
                    Label("Start Workout", systemImage: "play.fill")
                        .font(.system(size: 16, weight: .bold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(14)
                }

            case .active:
                Button(action: pauseWorkout) {
                    Image(systemName: "pause.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.white.opacity(0.12))
                        .foregroundColor(.white)
                        .cornerRadius(14)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(width: 70)
                        .padding(.vertical, 14)
                        .background(Color.red.opacity(0.8))
                        .foregroundColor(.white)
                        .cornerRadius(14)
                }

            case .paused:
                Button(action: resumeWorkout) {
                    Image(systemName: "play.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(14)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Image(systemName: "stop.fill")
                        .font(.system(size: 18, weight: .bold))
                        .frame(width: 70)
                        .padding(.vertical, 14)
                        .background(Color.red.opacity(0.8))
                        .foregroundColor(.white)
                        .cornerRadius(14)
                }
            }
        }
        .padding(.horizontal, 12)
    }

    // Prefer Watch HR, fall back to HealthKit
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
        let hours = Int(interval) / 3600
        let minutes = (Int(interval) % 3600) / 60
        let seconds = Int(interval) % 60
        if hours > 0 {
            return String(format: "%d:%02d:%02d", hours, minutes, seconds)
        }
        return String(format: "%02d:%02d", minutes, seconds)
    }
}
