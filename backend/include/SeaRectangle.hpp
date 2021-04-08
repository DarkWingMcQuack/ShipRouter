#include <LatLng.hpp>
#include <algorithm>
#include <array>

class SeaRectangle
{
public:
    constexpr SeaRectangle(const Latitude<Degree>& top_left_lat,
                           const Longitude<Degree>& top_left_lng,
                           const Latitude<Degree>& bottom_right_lat,
                           const Longitude<Degree>& bottom_right_lng) noexcept
        : top_left_lat_(top_left_lat.normalizeDegree()),
          top_left_lng_(top_left_lng.normalizeDegree()),
          bottom_right_lat_(bottom_right_lat.normalizeDegree()),
          bottom_right_lng_(bottom_right_lng.normalizeDegree()) {}

    constexpr auto isIn(const Latitude<Degree>& lat,
                        const Longitude<Degree>& lng) noexcept
    {
        return top_left_lat_ > lat
            and top_left_lng_ < lng
            and bottom_right_lat_ < lat
            and bottom_right_lng_ > lng;
    }

private:
    Latitude<Degree> top_left_lat_;
    Longitude<Degree> top_left_lng_;
    Latitude<Degree> bottom_right_lat_;
    Longitude<Degree> bottom_right_lng_;
};

constexpr auto SEA_RECTANGLES = std::array{
    SeaRectangle{
        Latitude<Degree>{49.724479188712984},
        Longitude<Degree>{-201.796875},
        Latitude<Degree>{29.84064389983441},
        Longitude<Degree>{-128.671875}},

    SeaRectangle{
        Latitude<Degree>{37.50972584293751},
        Longitude<Degree>{-60.55664062499999},
        Latitude<Degree>{15.623036831528264},
        Longitude<Degree>{-26.455078125}},

    SeaRectangle{
        Latitude<Degree>{16.46769474828897},
        Longitude<Degree>{-58.71093750000001},
        Latitude<Degree>{8.581021215641854},
        Longitude<Degree>{-26.19140625}},

    SeaRectangle{
        Latitude<Degree>{-22.43134015636061},
        Longitude<Degree>{48.8671875},
        Latitude<Degree>{-36.949891786813275},
        Longitude<Degree>{112.412109375}},

    SeaRectangle{
        Latitude<Degree>{58.95000823335702},
        Longitude<Degree>{308.759765625},
        Latitude<Degree>{40.245991504199026},
        Longitude<Degree>{345.5859375}},

    SeaRectangle{
        Latitude<Degree>{18.312810846425442},
        Longitude<Degree>{417.216796875},
        Latitude<Degree>{-14.519780046326085},
        Longitude<Degree>{430.6640625}},

    SeaRectangle{
        Latitude<Degree>{3.337953961416485},
        Longitude<Degree>{438.3984375},
        Latitude<Degree>{-63.78248603116501},
        Longitude<Degree>{454.74609375}},

    SeaRectangle{
        Latitude<Degree>{15.961329081596647},
        Longitude<Degree>{173.14453125},
        Latitude<Degree>{10.833305983642491},
        Longitude<Degree>{260.859375}},

    SeaRectangle{
        Latitude<Degree>{62.34960927573042},
        Longitude<Degree>{318.60351562499994},
        Latitude<Degree>{40.111688665595956},
        Longitude<Degree>{345.498046875}},

    SeaRectangle{
        Latitude<Degree>{57.136239319177434},
        Longitude<Degree>{307.79296875},
        Latitude<Degree>{40.111688665595956},
        Longitude<Degree>{348.662109375}},

    SeaRectangle{
        Latitude<Degree>{-28.459033019728043},
        Longitude<Degree>{186.6796875},
        Latitude<Degree>{-68.13885164925573},
        Longitude<Degree>{277.91015625}},

    SeaRectangle{
        Latitude<Degree>{-25.720735134412095},
        Longitude<Degree>{34.716796875},
        Latitude<Degree>{-45.70617928533083},
        Longitude<Degree>{76.904296875}},

    SeaRectangle{
        Latitude<Degree>{10.055402736564236},
        Longitude<Degree>{-134.12109375},
        Latitude<Degree>{-26.11598592533351},
        Longitude<Degree>{-92.46093749999999}},

    SeaRectangle{
        Latitude<Degree>{-8.407168163601076},
        Longitude<Degree>{-34.62890625},
        Latitude<Degree>{-15.623036831528252},
        Longitude<Degree>{11.6015625}},

    SeaRectangle{
        Latitude<Degree>{-17.056784609942543},
        Longitude<Degree>{-28.388671875},
        Latitude<Degree>{10.458984375},
        Longitude<Degree>{-36.52729481454623}},

    SeaRectangle{
        Latitude<Degree>{22.43134015636061},
        Longitude<Degree>{483.662109375},
        Latitude<Degree>{15.114552871944115},
        Longitude<Degree>{494.912109375}},

    SeaRectangle{
        Latitude<Degree>{61.95961583829658},
        Longitude<Degree>{-92.2412109375},
        Latitude<Degree>{57.326521225217064},
        Longitude<Degree>{-81.1669921875}},
};

constexpr auto isDefinitlySea(const Latitude<Degree>& lat,
                              const Longitude<Degree>& lng)
{
    for(auto rec : SEA_RECTANGLES) {
        if(rec.isIn(lat, lng)) {
            return true;
        }
    }
    return false;
}
