# A small program for generating the sine table for fastSine.cpp

# Imports
import math

# Constants
trailingZeros = 0
#PWM_MAX_DUTY_CYCLE = 4095  # 2**12 - 1

# Main function
def main():

    # Ask for the microstep divisions
    try:
        divisor = int(input("Enter the maximum supported divisor (number under the 1/, such as 32 for 1/32): "))
    except Exception:
        Exception("Invalid divisor. Divisor must be an integer.")

    # Ask for the number of elements per line
    try:
        valuesPerLine = int(input("Enter the number of values per line: "))
    except Exception:
        Exception("Invalid values per line. Number of values must be an integer.")

    # Ask for the number of elements per line
    try:
        sineMax = int(input("Enter the maximum sine value: "))
    except Exception:
        Exception("Invalid maximum sine value. The maximum must be an integer.")

    # Create a master string for storing the final print
    finalString = ""

    # Add the initial datatype and start of bracket
    finalString += "static const float sineTable[" + str(4 * divisor) + "] = {\n"

    # A counter for the current value that is being evaluated by the while loop
    counter = 0

    # Loop through, generating the sine values and printing them.
    while (((360 * counter) / (4 * divisor)) < 360):

        # Calculate the sine with the degree value
        value = math.trunc(math.sin(math.radians((360 * counter) / (4 * divisor))) * sineMax)

        # Add a space if the value is positive or negative sign if value is negative, but don't pad the first values
        if (value >= 0):
            sign = " "
        else:
            sign = "-"

        # Pad the leading spaces
        if (abs(value) < 10):
            
            # Need 4 spaces
            spaces = "    "
        
        elif (abs(value) < 100):

            # Need 3 spaces
            spaces = "   "

        elif(abs(value) < 1000):
            
            # Need 2 spaces
            spaces = "  "

        elif(abs(value) < 10000):

            # Need one space
            spaces = " "

        else:
            # Set spaces to none
            spaces = ""


        # Pad the value and add it to the final string
        #finalString += (sign + str(abs(value)) + ('0' * ((trailingZeros + 2) - len(str(abs(value))))))
        finalString += spaces + sign + str(abs(value))
        
        # Check if the next value should have a comma. Prevents the last value from having an extra comma
        if (((360 * (counter + 1)) / (4 * divisor)) < 360):
            finalString += ","

        # Add a newline if needed
        if ((counter + 1) % valuesPerLine == 0 and counter != 0):
            finalString += "\n"
        else:
            finalString += " "

        # Increment the current value
        counter += 1

    # Add the ending bracket
    finalString += "};\n"

    # Print out the string that was created
    print(finalString)






# Run the function if the program is called independently
if __name__ == "__main__":
    main()
