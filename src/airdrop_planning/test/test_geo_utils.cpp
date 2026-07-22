// Guard kontrak konversi yang diandalkan mission_bridge:
// titik NED yang dihitung solver -> lat/lon -> kembali ke NED harus konsisten.
#include <gtest/gtest.h>
#include "airdrop_planning/geo_utils.hpp"

using airdrop_planning::LocalFrame;
using airdrop_planning::Ned2D;
using airdrop_planning::GeoPoint;

TEST(GeoUtils, NedToGeoRoundtrip)
{
  LocalFrame frame;
  frame.setDatum(-6.914744, 107.609810);  // contoh datum (Bandung)

  const Ned2D p{123.4, -56.7};            // release point [m] NED
  const GeoPoint g = frame.toGeo(p);
  const Ned2D back = frame.toNed(g);

  EXPECT_NEAR(back.north, p.north, 1e-3);
  EXPECT_NEAR(back.east, p.east, 1e-3);
  // lat/lon harus bergerak ke arah yang benar dari datum
  EXPECT_LT(g.lat_deg, -6.914744 + 1.0);
  EXPECT_GT(g.lat_deg, -6.914744);        // north positif -> lat naik
}
