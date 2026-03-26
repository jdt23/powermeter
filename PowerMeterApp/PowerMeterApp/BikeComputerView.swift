import SwiftUI
import Combine

struct BikeComputerView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @EnvironmentObject var connectivityManager: PhoneConnectivityManager

    var onWorkoutEnd: () -> Void

    @State private var showingStopConfirmation = false

    // Record sensor data periodically
    @State private var recordingCancellable: AnyCancellable?

    var body: some View {
        VStack(spacing: 0) {
            // Connection status bar
            connectionBar

            // Main metrics grid
            metricsGrid
                .padding(.horizontal, 12)
                .padding(.top, 8)

            Spacer()

            // Workout controls
            workoutControls
                .padding(.bottom, 20)
        }
        .background(Color.black)
        .onAppear { startRecording() }
        .onDisappear { recordingCancellable?.cancel() }
        .confirmationDialog("End Workout?", isPresented: $showingStopConfirmation) {
            Button("End Workout", role: .destructive) { endWorkout() }
            Button("Cancel", role: .cancel) {}
        }
    }

    // MARK: - Connection Bar

    private var connectionBar: some View {
        HStack {
            Circle()
                .fill(connectionColor)
                .frame(width: 10, height: 10)
            Text(bleManager.connectionState.rawValue)
                .font(.caption)
                .foregroundColor(.gray)
            if connectivityManager.isWatchReachable {
                Image(systemName: "applewatch.radiowaves.left.and.right")
                    .font(.caption2)
                    .foregroundColor(.green)
            }
            Spacer()
            if workoutSession.state == .active {
                Text(formatDuration(workoutSession.elapsed))
                    .font(.system(.title3, design: .monospaced))
                    .foregroundColor(.white)
                    .fontWeight(.semibold)
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .background(Color.black)
    }

    private var connectionColor: Color {
        switch bleManager.connectionState {
        case .connected: return .green
        case .scanning, .connecting: return .yellow
        case .disconnected: return .red
        }
    }

    // MARK: - Metrics Grid

    private var metricsGrid: some View {
        VStack(spacing: 8) {
            // Power - full width, large
            MetricCard(
                label: "POWER",
                value: "\(bleManager.power)",
                unit: "W",
                color: powerColor,
                style: .hero
            )

            HStack(spacing: 8) {
                MetricCard(
                    label: "CADENCE",
                    value: "\(bleManager.cadence)",
                    unit: "RPM",
                    color: .cyan
                )
                MetricCard(
                    label: "RESISTANCE",
                    value: "\(bleManager.resistance)",
                    unit: "",
                    color: .orange
                )
            }

            HStack(spacing: 8) {
                MetricCard(
                    label: "HEART RATE",
                    value: currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--",
                    unit: "BPM",
                    color: .red
                )
                MetricCard(
                    label: "CALORIES",
                    value: "\(Int(workoutSession.totalCalories))",
                    unit: "KCAL",
                    color: .yellow
                )
            }

            if workoutSession.state != .idle {
                HStack(spacing: 8) {
                    MetricCard(
                        label: "AVG POWER",
                        value: "\(Int(workoutSession.averagePower))",
                        unit: "W",
                        color: .purple
                    )
                    MetricCard(
                        label: "AVG CADENCE",
                        value: "\(Int(workoutSession.averageCadence))",
                        unit: "RPM",
                        color: .cyan.opacity(0.7)
                    )
                }

                if workoutSession.averageHeartRate > 0 {
                    HStack(spacing: 8) {
                        MetricCard(
                            label: "AVG HR",
                            value: "\(Int(workoutSession.averageHeartRate))",
                            unit: "BPM",
                            color: .red.opacity(0.7)
                        )
                        MetricCard(
                            label: "MAX HR",
                            value: "\(Int(workoutSession.maxHeartRate))",
                            unit: "BPM",
                            color: .red
                        )
                    }
                }
            }
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

    // MARK: - Workout Controls

    private var workoutControls: some View {
        HStack(spacing: 20) {
            switch workoutSession.state {
            case .idle:
                Button(action: startWorkout) {
                    Label("Start Workout", systemImage: "play.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(12)
                }

            case .active:
                Button(action: pauseWorkout) {
                    Label("Pause", systemImage: "pause.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(Color.yellow)
                        .foregroundColor(.black)
                        .cornerRadius(12)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Label("Stop", systemImage: "stop.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(Color.red)
                        .foregroundColor(.white)
                        .cornerRadius(12)
                }

            case .paused:
                Button(action: resumeWorkout) {
                    Label("Resume", systemImage: "play.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(12)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Label("Stop", systemImage: "stop.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(Color.red)
                        .foregroundColor(.white)
                        .cornerRadius(12)
                }
            }
        }
        .padding(.horizontal, 16)
    }

    // Prefer Watch HR (real-time from workout session), fall back to HealthKit
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

                // Send metrics to Watch for display
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

    // MARK: - Helpers

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

// MARK: - Metric Card

enum MetricStyle {
    case standard
    case hero
}

struct MetricCard: View {
    let label: String
    let value: String
    let unit: String
    let color: Color
    var style: MetricStyle = .standard

    var body: some View {
        VStack(spacing: 4) {
            Text(label)
                .font(.caption2)
                .fontWeight(.medium)
                .foregroundColor(color.opacity(0.8))
                .tracking(1)

            HStack(alignment: .lastTextBaseline, spacing: 4) {
                Text(value)
                    .font(.system(size: style == .hero ? 64 : 40, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
                    .minimumScaleFactor(0.5)
                    .lineLimit(1)

                if !unit.isEmpty {
                    Text(unit)
                        .font(style == .hero ? .title3 : .caption)
                        .foregroundColor(.gray)
                }
            }
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, style == .hero ? 16 : 12)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(Color.white.opacity(0.05))
                .overlay(
                    RoundedRectangle(cornerRadius: 12)
                        .strokeBorder(color.opacity(0.3), lineWidth: 1)
                )
        )
    }
}
