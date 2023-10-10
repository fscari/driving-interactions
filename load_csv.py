import pandas as pd

# Specify the path to your CSV file
file_path = "/home/fede/PycharmProjects/driving-interactions/Experiment Nr_2/Experiment Nr_2_Condition_AV left_Iteration Nr_1.csv"

# Load the CSV data into a DataFrame
data = pd.read_csv(file_path)

# Display the first few rows of the DataFrame
print(data.head())
