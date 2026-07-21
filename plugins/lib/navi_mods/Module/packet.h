#pragma once
#include <cstdint>
#include <vector>
#include "types.h"

namespace navi {

    template <DataType DT, DataSource DS, ScalarType ST>
    struct _Payload {

    public:
        double time;
        std::vector<int64_t> dims;
        std::vector<char> data;

        DataType data_type() { return dt; }
        DataSource data_source() { return ds; }
        ScalarType scalar_type() { return scalar; }

    private:
        DataType dt = DT;
        DataSource ds = DS;
        ScalarType scalar = ST;

    public:
        template <class S>
        void serialize(S &s)
        {
            s(dt);
            s(ds);
            s(scalar);
            s(dims);
            s(data);
            s(time);
        }

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &dt;
            ar &ds;
            ar &scalar;
            ar &dims;
            ar &data;
            ar &time;
        }
    };

    template <typename T>
    using Payload = _Payload<T::data_type, T::data_source, T::scalar_type>;

} // namespace navi
