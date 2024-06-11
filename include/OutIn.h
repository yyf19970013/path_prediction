class OutIn
{
private:
    
public:
    OutIn() = default;
    ~OutIn() = default;

    bool HasV(int ID);
    pair<double,double> GetTime(const coordinate& st, const coordinate& end);

};
