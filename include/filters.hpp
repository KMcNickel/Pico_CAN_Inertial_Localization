

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

        float addNewData(float data)
        {
            currentValue = currentValue - (beta * (currentValue - data));
            return currentValue;
        }

        float getCurrentValue()
        {
            return currentValue;
        }

    private:
        float currentValue;
        float beta = 0.5;
};