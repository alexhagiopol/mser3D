//
// Created by alex on 3/13/16.
//

#include <gtsam/base/Manifold.h>

#pragma once
namespace gtsam{
    template<typename M1, typename M2, typename M3>
    class TripleManifold: public std::tuple<M1,M2,M3> {
        BOOST_CONCEPT_ASSERT((IsManifold<M1>));
        BOOST_CONCEPT_ASSERT((IsManifold<M2>));
        BOOST_CONCEPT_ASSERT((IsManifold<M3>));

    protected:
        enum { dimension1 = traits<M1>::dimension };
        enum { dimension2 = traits<M2>::dimension };
        enum { dimension3 = traits<M3>::dimension };

    public:
        enum { dimension = dimension1 + dimension2 + dimension3};
        inline static size_t Dim() { return dimension;}
        inline size_t dim() const { return dimension;}

        typedef Eigen::Matrix<double, dimension, 1> TangentVector;
        typedef OptionalJacobian<dimension, dimension> ChartJacobian;

        /// Default constructor needs default constructors to be defined
        TripleManifold():std::tuple<M1,M2,M3>(M1(),M2(),M3()) {}

        // Construct from three original manifold values
        TripleManifold(const M1& m1, const M2& m2, const M3& m3):std::tuple<M1,M2,M3>(m1,m2,m3) {}

        /// Retract delta to manifold
        TripleManifold retract(const TangentVector& xi) const {
            M1 m1 = traits<M1>::Retract(get<0>(*this), xi.template head<dimension1>());
            M2 m2 = traits<M2>::Retract(get<1>(*this), xi.template segment<dimension2>(dimension1)); //get dimension2 items starting at index dimension1
            M3 m3 = traits<M3>::Retract(get<2>(*this), xi.template tail<dimension3>());
            return TripleManifold(m1,m2,m3);
        }

        /// Compute the coordinates in the tangent space
        TangentVector localCoordinates(const TripleManifold& other) const {
            typename traits<M1>::TangentVector v1 = traits<M1>::Local(get<0>(*this),  get<0>(other));
            typename traits<M2>::TangentVector v2 = traits<M2>::Local(get<1>(*this),  get<1>(other));
            typename traits<M3>::TangentVector v3 = traits<M3>::Local(get<2>(*this),  get<2>(other));
            TangentVector v;
            v << v1, v2, v3;
            return v;
        }
    };

    // Define any direct product group to be a model of the multiplicative Group concept
    template<typename M1, typename M2, typename M3>
    struct traits<TripleManifold<M1, M2, M3>> : internal::Manifold<TripleManifold<M1, M2, M3> > {
    };
}
