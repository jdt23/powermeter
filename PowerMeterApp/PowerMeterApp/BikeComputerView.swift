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
        VStack(spacing: 0) {
            connectionBar
            metricsGrid
                .padding(.horizontal, 12)
                .padding(.top, 4)
            Spacer(minLength: 8)
            workoutControls
                .padding(.bottom, 16)
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
            if workoutSession.state != .idle {
                Text(formatDuration(workoutSession.elapsed))
                    .font(.system(.title3, design: .monospaced))
                    .foregroundColor(.white)
                    .fontWeight(.semibold)
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 6)
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
        let isWorkout = workoutSession.state != .idle
        return VStack(spacing: 6) {
            // Power - hero metric with avg underneath
            LiveMetricCard(
                label: "POWER",
                value: "\(bleManager.power)",
                unit: "W",
                color: powerColor,
                avg: isWorkout ? "avg \(Int(workoutSession.averagePower))" : nil,
                max: isWorkout && workoutSession.maxPower > 0 ? "max \(Int(workoutSession.maxPower))" : nil,
                style: .hero
            )

            HStack(spacing: 6) {
                LiveMetricCard(
                    label: "CADENCE",
                    value: "\(bleManager.cadence)",
                    unit: "RPM",
                    color: .cyan,
                    avg: isWorkout ? "avg \(Int(workoutSession.averageCadence))" : nil
                )
                LiveMetricCard(
                    label: "RESISTANCE",
                    value: "\(bleManager.resistance)",
                    unit: "",
                    color: .orange
                )
            }

            HStack(spacing: 6) {
                LiveMetricCard(
                    label: "HEART RATE",
                    value: currentHeartRate > 0 ? "\(Int(currentHeartRate))" : "--",
                    unit: "BPM",
                    color: .red,
                    avg: isWorkout && workoutSession.averageHeartRate > 0 ? "avg \(Int(workoutSession.averageHeartRate))" : nil,
                    max: isWorkout && workoutSession.maxHeartRate > 0 ? "max \(Int(workoutSession.maxHeartRate))" : nil
                )
                LiveMetricCard(
                    label: "CALORIES",
                    value: "\(Int(workoutSession.totalCalories))",
                    unit: "KCAL",
                    color: .yellow
                )
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
        HStack(spacing: 16) {
            switch workoutSession.state {
            case .idle:
                Button(action: startWorkout) {
                    Label("Start Workout", systemImage: "play.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(12)
                }

            case .active:
                Button(action: pauseWorkout) {
                    Label("Pause", systemImage: "pause.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.yellow)
                        .foregroundColor(.black)
                        .cornerRadius(12)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Label("Stop", systemImage: "stop.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.red)
                        .foregroundColor(.white)
                        .cornerRadius(12)
                }

            case .paused:
                Button(action: resumeWorkout) {
                    Label("Resume", systemImage: "play.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.green)
                        .foregroundColor(.black)
                        .cornerRadius(12)
                }
                Button(action: { showingStopConfirmation = true }) {
                    Label("Stop", systemImage: "stop.fill")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.red)
                        .foregroundColor(.white)
                        .cornerRadius(12)
                }
            }
        }
        .padding(.horizontal, 16)
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

// MARK: - Live Metric Card (instantaneous large, avg/max small)

enum MetricStyle {
    case standard
    case hero
}

struct LiveMetricCard: View {
    let label: String
    let value: String
    let unit: String
    let color: Color
    var avg: String? = nil
    var max: String? = nil
    var style: MetricStyle = .standard

    var body: some View {
        VStack(spacing: 2) {
            Text(label)
                .font(.system(size: 10, weight: .semibold))
                .foregroundColor(color.opacity(0.8))
                .tracking(1.5)

            HStack(alignment: .lastTextBaseline, spacing: 3) {
                Text(value)
                    .font(.system(size: style == .hero ? 72 : 44, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
                    .minimumScaleFactor(0.5)
                    .lineLimit(1)

                if !unit.isEmpty {
                    Text(unit)
                        .font(.system(size: style == .hero ? 20 : 13, weight: .medium))
                        .foregroundColor(.gray)
                        .padding(.bottom, style == .hero ? 6 : 2)
                }
            }

            // Avg / Max row - smaller, muted
            if avg != nil || max != nil {
                HStack(spacing: 10) {
                    if let avg {
                        Text(avg)
                            .font(.system(size: 12, weight: .medium, design: .rounded))
                            .foregroundColor(color.opacity(0.5))
                    }
                    if let max {
                        Text(max)
                            .font(.system(size: 12, weight: .medium, design: .rounded))
                            .foregroundColor(color.opacity(0.5))
                    }
                }
                .padding(.top, -2)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, style == .hero ? 12 : 8)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(Color.white.opacity(0.05))
                .overlay(
                    RoundedRectangle(cornerRadius: 12)
                        .strokeBorder(color.opacity(0.2), lineWidth: 1)
                )
        )
    }
}
