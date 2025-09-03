# Imports:
# --------
import json
import numpy as np


# Function:
# ---------
def get_empty_dict():
    """
    Returns an empty dictionary with keys for x, y, z, roll, pitch, yaw initialized to 0.0.
    
    return: dictionary with keys "x", "y", "z", "roll", "pitch", "yaw"
    """
    return {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0
    }


# Function:
# ---------
def invert_T(T):
    """
    Inverts the given transformation matrix T.

    param T: 4x4 numpy array representing the transformation matrix

    return: 4x4 numpy array representing the inverted transformation matrix as Autoware expects v_T_sensor
    """
    R = T[:3,:3]

    t = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3,:3] = R.T
    T_inv[:3, 3] = -R.T @ t
    
    return T_inv


# Function: 
# ---------
def rpy_from_R(R):
    """
    Extracts roll, pitch, yaw from the given rotation matrix R.

    param R: 3x3 numpy array representing the rotation matrix

    return: roll, pitch, yaw in radians

    # NOTE: We use the extrinsic transormation matrix decomposition
    """
    
    cos_beta = np.sqrt(R[0,0]**2 + R[1,0]**2) # cos(ß)

    if cos_beta > 1e-6:
        # Without gimal lock:
        # -------------------
        yaw  = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], cos_beta)
        roll   = np.arctan2(R[1,0], R[0,0])
    else:
        # With gimbal lock: Not an issue for Autonomous driving
        # -----------------
        roll  = np.arctan2(-R[1,2], R[0,2])
        pitch = np.arctan2(-R[2,0], cos_beta)
        yaw   = 0.0
    
    return float(roll), float(pitch), float(yaw)


# Function:
# ---------
def xyz_rpy_from_T(v_T_sensor):
    """
    Decomposes the given transformation matrix T into translation (x,y,z) and 
    rotation (roll, pitch, yaw) and returns the results in a dictionary.

    param v_T_sensor: 4x4 numpy array representing the transformation matrix

    return: dictionary with keys "x", "y", "z", "roll", "pitch", "yaw"
    """
    # Step 1: Get translation information
    t = v_T_sensor[:3, 3]

    # Step 2: Get rotation information
    R = v_T_sensor[:3,:3]

    # Step 3: Extract roll, pitch, yaw from R
    r,p,y = rpy_from_R(R)

    # Step 4: Store the results in a dictionary
    data = get_empty_dict()
    
    data["x"] = float(t[0])
    data["y"] = float(t[1])
    data["z"] = float(t[2])
    data["roll"] = r
    data["pitch"] = p
    data["yaw"] = y

    return data


# Function: 
# ---------
def get_info(calibration_file_path: str="calibration_AudiQ8.yaml"):
    """
    Main function that reads the .yaml calibration file and decomposes 
    the T matrix for each sensor and writes the results into a new .json file.
    
    param calibration_file_path: path to the .yaml calibration file for Audi Q8
    
    return: None
    """

    # Data to write as .json file:
    # ----------------------------
    xyzrpy = {}

    # Read the calibration file and rearrange the data:
    # -------------------------------------------------
    with open(calibration_file_path) as file:
        items = json.load(file)

        for each_key in items:
            if each_key != "state":
                transform = list(items[each_key]['extrinsics'].keys())[0]

                (name, v_T_sensor) = (f"vTc_{each_key}", True) if transform == "cTv" else (f"{transform}_{each_key}", False)

                T = np.array(items[each_key]['extrinsics'][transform])

                T = invert_T(T) if v_T_sensor else T

                data = xyz_rpy_from_T(T)

                xyzrpy[name] = data

    # Write the results into a new .json file:
    # ----------------------------------------
    with open("calibration_AudiQ8_xyzrpy.json", "w") as outfile:
        json.dump({"Audi Q8": xyzrpy}, outfile, indent=4)
        
        print("Done writing to file: calibration_AudiQ8_xyzrpy.json")


# Run as script:
# --------------
if __name__ == "__main__":    
    get_info(calibration_file_path="calibration_AudiQ8.json")
    