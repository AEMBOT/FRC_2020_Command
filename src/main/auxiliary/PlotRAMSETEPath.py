"""When given a CSV file will plot the expected result over the actual result"""
import matplotlib.pyplot as plt

class CSVParser():
    """Reads and converts the generated CSV file to a dict"""
    @staticmethod
    def parseCSVFile(filePath):
        csvDict = {
            "realX": [],
            "realY": [],
            "expectedX": [],
            "expectedY": [],
        }
        with open(filePath) as f:
            for i, line in enumerate(f):
                if i != 0:
                    splitLine = line.split(",")
                    csvDict["realX"].append(float(splitLine[0].strip()))
                    csvDict["realY"].append(float(splitLine[1].strip()))
                    csvDict["expectedX"].append(float(splitLine[2].strip()))
                    csvDict["expectedY"].append(float(splitLine[3].strip()))

        return csvDict

if __name__ == "__main__":
    """Plots the CSV file data on a correctly sized plot"""
    filename = "OutputData.csv"
    plt.style.use('seaborn-whitegrid')
    parsedCSV = CSVParser.parseCSVFile(filename)
    print(parsedCSV)
    fig, ax = plt.subplots(figsize=(12, 6))

    ax.set_title("Real Values vs Calculated")
    ax.plot(parsedCSV["realX"], parsedCSV["realY"], color="red", label="Real Values")
    ax.plot(parsedCSV["expectedX"], parsedCSV["expectedY"], color="green", label="Calculated Values")
    plt.legend()
    plt.xlabel("X Position (Feet)")
    plt.ylabel("Y Position (Feet)")
    
    plt.xlim([0, 30])
    plt.ylim([0, 15])
    
    plt.show()