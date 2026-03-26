import SwiftUI

struct WorkoutView: View {
    @EnvironmentObject var workoutManager: WatchWorkoutManager
    @EnvironmentObject var connectivityManager: WatchConnectivityManager

    var body: some View {
        if workoutManager.isActive {
            activeWorkoutView
        } else {
            waitingView
        }
    }

    private var waitingView: some View {
        VStack(spacing: 12) {
            Image(systemName: "bicycle")
                .font(.system(size: 36))
                .foregroundColor(.green)

            Text("PowerMeter")
                .font(.headline)

            if connectivityManager.isPhoneReachable {
                Text("Start workout\non iPhone")
                    .font(.caption)
                    .foregroundColor(.gray)
                    .multilineTextAlignment(.center)
            } else {
                Text("Connecting to\niPhone...")
                    .font(.caption)
                    .foregroundColor(.yellow)
                    .multilineTextAlignment(.center)
            }
        }
    }

    private var activeWorkoutView: some View {
        ScrollView {
            VStack(spacing: 8) {
                // Duration
                Text(formatDuration(workoutManager.elapsed))
                    .font(.system(.title3, design: .monospaced))
                    .foregroundColor(.white)

                // Power - hero metric
                WatchMetric(label: "POWER", value: "\(workoutManager.power)", unit: "W", color: .green)

                HStack(spacing: 8) {
                    WatchMetricSmall(label: "CAD", value: "\(workoutManager.cadence)", color: .cyan)
                    WatchMetricSmall(label: "RES", value: "\(workoutManager.resistance)", color: .orange)
                }

                // Heart rate
                WatchMetric(
                    label: "HEART RATE",
                    value: workoutManager.heartRate > 0 ? "\(Int(workoutManager.heartRate))" : "--",
                    unit: "BPM",
                    color: .red
                )

                if workoutManager.activeCalories > 0 {
                    WatchMetricSmall(label: "CAL", value: "\(Int(workoutManager.activeCalories))", color: .yellow)
                }

                if workoutManager.isPaused {
                    Text("PAUSED")
                        .font(.caption)
                        .foregroundColor(.yellow)
                        .padding(.top, 4)
                }
            }
            .padding(.horizontal, 4)
        }
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

struct WatchMetric: View {
    let label: String
    let value: String
    let unit: String
    let color: Color

    var body: some View {
        VStack(spacing: 2) {
            Text(label)
                .font(.system(size: 10))
                .foregroundColor(color.opacity(0.8))
            HStack(alignment: .lastTextBaseline, spacing: 2) {
                Text(value)
                    .font(.system(size: 28, weight: .bold, design: .rounded))
                    .foregroundColor(.white)
                    .minimumScaleFactor(0.6)
                    .lineLimit(1)
                Text(unit)
                    .font(.system(size: 10))
                    .foregroundColor(.gray)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 6)
        .background(
            RoundedRectangle(cornerRadius: 8)
                .fill(Color.white.opacity(0.07))
        )
    }
}

struct WatchMetricSmall: View {
    let label: String
    let value: String
    let color: Color

    var body: some View {
        VStack(spacing: 1) {
            Text(label)
                .font(.system(size: 9))
                .foregroundColor(color.opacity(0.8))
            Text(value)
                .font(.system(size: 22, weight: .bold, design: .rounded))
                .foregroundColor(.white)
                .minimumScaleFactor(0.6)
                .lineLimit(1)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 4)
        .background(
            RoundedRectangle(cornerRadius: 8)
                .fill(Color.white.opacity(0.07))
        )
    }
}
