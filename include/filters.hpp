

class LowPassFilter
{
    public:
        LowPassFilter() { }

        LowPassFilter(float b)
        {
            beta = b;
        }

        void setBeta(float b)
        {
            beta = b;
        }

        void setInitialValue(float data)
        {
            currentValue = data;
        }

        float addNewData(float data)
        {
            currentValue = currentValue - (beta * (currentValue - data));
            return currentValue;
        }

        void addNewData(float * data)
        {
            currentValue = currentValue - (beta * (currentValue - *data));
            *data = currentValue;
        }

        float getCurrentValue()
        {
            return currentValue;
        }

        float * CurrentValue()
        {
            return &currentValue;
        }

    private:
        float currentValue;
        float beta = 0.5;
};