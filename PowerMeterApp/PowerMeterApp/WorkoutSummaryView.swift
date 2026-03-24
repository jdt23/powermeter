import SwiftUI

struct WorkoutSummaryView: View {
    let summary: WorkoutSummary
    let onDismiss: () -> Void

    var body: some View {
        VStack(spacing: 24) {
            Text("Workout Complete")
                .font(.largeTitle)
                .fontWeight(.bold)
                .foregroundColor(.white)

            VStack(spacing: 16) {
                summaryRow("Duration", value: formatDuration(summary.duration))
                Divider().background(Color.gray)
                summaryRow("Avg Power", value: "\(Int(summary.averagePower)) W")
                summaryRow("Max Power", value: "\(Int(summary.maxPower)) W")
                Divider().background(Color.gray)
                summaryRow("Avg Cadence", value: "\(Int(summary.averageCadence)) RPM")
                Divider().background(Color.gray)
                if summary.averageHeartRate > 0 {
                    summaryRow("Avg Heart Rate", value: "\(Int(summary.averageHeartRate)) BPM")
                    summaryRow("Max Heart Rate", value: "\(Int(summary.maxHeartRate)) BPM")
                    Divider().background(Color.gray)
                }
                summaryRow("Calories", value: "\(Int(summary.totalCalories)) kcal")
            }
            .padding(24)
            .background(
                RoundedRectangle(cornerRadius: 16)
                    .fill(Color.white.opacity(0.05))
            )

            Text("Saved to Apple Health")
                .font(.subheadline)
                .foregroundColor(.green)

            Button(action: onDismiss) {
                Text("Done")
                    .font(.headline)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(Color.blue)
                    .foregroundColor(.white)
                    .cornerRadius(12)
            }
        }
        .padding(24)
    }

    private func summaryRow(_ label: String, value: String) -> some View {
        HStack {
            Text(label)
                .foregroundColor(.gray)
            Spacer()
            Text(value)
                .fontWeight(.semibold)
                .foregroundColor(.white)
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
