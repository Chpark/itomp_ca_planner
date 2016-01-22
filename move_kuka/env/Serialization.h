
	/**
	Serialization and deserilization of a 4 x 4 homogenous matrix.
	*/
	template<class Archive>
	void serialize(Archive & ar, HMatrix& m, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("r11", m[0]);
		ar & boost::serialization::make_nvp("r12", m[1]);
		ar & boost::serialization::make_nvp("r13", m[2]);
		ar & boost::serialization::make_nvp("r14", m[3]);

		ar & boost::serialization::make_nvp("r21", m[4]);
		ar & boost::serialization::make_nvp("r22", m[5]);
		ar & boost::serialization::make_nvp("r23", m[6]);
		ar & boost::serialization::make_nvp("r24", m[7]);

		ar & boost::serialization::make_nvp("r31", m[8]);
		ar & boost::serialization::make_nvp("r32", m[9]);
		ar & boost::serialization::make_nvp("r33", m[10]);
		ar & boost::serialization::make_nvp("r34", m[11]);

		ar & boost::serialization::make_nvp("r41", m[12]);
		ar & boost::serialization::make_nvp("r42", m[13]);
		ar & boost::serialization::make_nvp("r43", m[14]);
		ar & boost::serialization::make_nvp("r44", m[15]);

	}
