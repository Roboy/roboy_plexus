//
// ResultMetadata.h
//
// Library: Data/MySQL
// Package: MySQL
// Module:  ResultMetadata
//
// Definition of the ResultMetadata class.
//
// Copyright (c) 2008, Applied Informatics Software Engineering GmbH.
// and Contributors.
//
// SPDX-License-Identifier:	BSL-1.0
//


#ifndef Data_MySQL_ResultMetadata_INCLUDED
#define Data_MySQL_ResultMetadata_INCLUDED

#include <mysql.h>
#include <vector>
#include "Poco/Data/MetaColumn.h"

namespace Poco {
namespace Data {
namespace MySQL {

#if (MYSQL_VERSION_ID >= 80001) && !defined(MARIADB_BASE_VERSION)
struct bool_wrapper {
	bool inner_bool;
};
#endif

class ResultMetadata
	/// MySQL result metadata
{
public:

	void reset();
		/// Resets the metadata.

	void init(MYSQL_STMT* stmt);
		/// Initializes the metadata.

	std::size_t columnsReturned() const;
		/// Returns the number of columns in resultset.

	const MetaColumn& metaColumn(std::size_t pos) const;
		/// Returns the reference to the specified metacolumn.

	MYSQL_BIND* row();
		/// Returns pointer to native row.

	std::size_t length(std::size_t pos) const;
		/// Returns the length.

	const unsigned char* rawData(std::size_t pos) const;
		/// Returns raw data.

	bool isNull(std::size_t pos) const;
		/// Returns true if value at pos is null.

private:
	std::vector<MetaColumn>    _columns;
	std::vector<MYSQL_BIND>    _row;
	std::vector<char>          _buffer;
	std::vector<unsigned long> _lengths;
#if (MYSQL_VERSION_ID >= 80001) && !defined(MARIADB_BASE_VERSION)
	std::vector<struct bool_wrapper>          _isNull;
#else
	std::vector<my_bool>       _isNull;
#endif
};

}}}

#endif //Data_MySQL_ResultMetadata_INCLUDED
