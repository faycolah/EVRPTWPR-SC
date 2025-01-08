from os import PathLike
import pandas as pd
import re


# Define a class to store vehicle information and table data
class VRPData:
    def __init__(
        self,
        vehicle_fuel_capacity: float,
        vehicle_load_capacity: float,
        fuel_consumption_rate: float,
        inverse_refueling_rate: float,
        average_velocity: float,
        data: pd.DataFrame,
    ) -> None:
        self.vehicle_fuel_capacity = vehicle_fuel_capacity
        self.vehicle_load_capacity = vehicle_load_capacity
        self.fuel_consumption_rate = fuel_consumption_rate
        self.inverse_refueling_rate = inverse_refueling_rate
        self.average_velocity = average_velocity
        self.data = data

    @property
    def set_of_customers(self) -> set[str]:
        return set(self.data[self.data["Type"] == "c"]["StringID"])

    @property
    def set_of_charging_stations(self) -> set[str]:
        return set(self.data[self.data["Type"] == "f"]["StringID"])

    @property
    def set_of_depots(self) -> set[str]:
        return set(self.data[self.data["Type"] == "d"]["StringID"])

    @property
    def set_of_nodes(self) -> set[str]:
        return set(self.data["StringID"])

    @property
    def coordinates_of_nodes(self) -> dict[str, tuple[float, float]]:
        return {row["StringID"]: (float(row["x"]), float(row["y"])) for _, row in self.data.iterrows()}

    @property
    def demand_of_nodes(self) -> dict[str, float]:
        return {row["StringID"]: float(row["demand"]) for _, row in self.data.iterrows()}

    @property
    def service_time(self) -> dict[str, float]:
        return {row["StringID"]: float(row["ServiceTime"]) for _, row in self.data.iterrows()}

    @property
    def time_window_of_nodes(self) -> dict[str, tuple[float, float]]:
        return {row["StringID"]: (float(row["ReadyTime"]), float(row["DueDate"])) for _, row in self.data.iterrows()}
    
    @property
    def latest_arrival_time_at_depot(self) -> float:
        return max(float(row["DueDate"]) for _, row in self.data.iterrows() if row["Type"] == "d")

    def __repr__(self) -> str:
        return (
            f"VRPData(\n"
            f"  vehicle_fuel_capacity={self.vehicle_fuel_capacity},\n"
            f"  vehicle_load_capacity={self.vehicle_load_capacity},\n"
            f"  fuel_consumption_rate={self.fuel_consumption_rate},\n"
            f"  inverse_refueling_rate={self.inverse_refueling_rate},\n"
            f"  average_velocity={self.average_velocity},\n"
            f"  data={self.data}\n"
            f")"
            )

def parse_instance_file(path_to_file: PathLike[str]) -> VRPData:
    # Parse the file
    
    with open(path_to_file, "r") as file:
        lines = file.readlines()

    # Extract vehicle information using regex and remove those lines
    file_content = "\n".join(lines)

    # Regex patterns for constants
    patterns = {
        "vehicle_fuel_capacity": r"Q Vehicle fuel tank capacity /([\d.]+)/",
        "vehicle_load_capacity": r"C Vehicle load capacity /([\d.]+)/",
        "fuel_consumption_rate": r"r fuel consumption rate /([\d.]+)/",
        "inverse_refueling_rate": r"g inverse refueling rate /([\d.]+)/",
        "average_velocity": r"v average Velocity /([\d.]+)/",
    }

    # Extract values and remove matched lines
    vehicle_info = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, file_content)
        if match:
            vehicle_info[key] = float(match.group(1))
            # Remove the matched line from lines
            lines = [line for line in lines if not re.search(pattern, line)]

    # Extract remaining table data
    header_line = lines[0].strip().split()
    data_lines = [line.strip().split() for line in lines[1:] if line.strip()]

    # Create a pandas DataFrame
    data = pd.DataFrame(data_lines, columns=header_line)

    # Create an instance of VRPData
    vrp_data = VRPData(
        vehicle_fuel_capacity=vehicle_info["vehicle_fuel_capacity"],
        vehicle_load_capacity=vehicle_info["vehicle_load_capacity"],
        fuel_consumption_rate=vehicle_info["fuel_consumption_rate"],
        inverse_refueling_rate=vehicle_info["inverse_refueling_rate"],
        average_velocity=vehicle_info["average_velocity"],
        data=data,
    )

    return vrp_data


if __name__ == "__main__":
    # Example usage
    file_path = "c101C5.txt"
    vrp_data = parse_instance_file(file_path)
    print(vrp_data)

