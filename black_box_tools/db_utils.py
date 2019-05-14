import subprocess
import pymongo as pm

class DBUtils(object):
    @staticmethod
    def restore_db(data_dir, drop_existing_records=True, db_name=None):
        '''Restores a MongoDB database dumped in the given directory. If "db_name"
        is not passed or is set to None, the name of the restored database
        will be the same as the name of the directory. Returns True if
        everything goes well; returns False otherwise.

        Keyword arguments:
        @param data_dir -- absolute path of a directory containing a dumped MongoDB database
        @param drop_existing_records -- indicates whether to drop any existing records
                                        if a database with the same name already
                                        exists (default True)
        @param db_name -- name of the database for restoring the data (default None,
                          in which case the database name is the same as the name
                          of the data directory)

        '''
        try:
            commands = ['mongorestore', data_dir]
            if drop_existing_records:
                commands.append('--drop')

            if db_name:
                commands.append('--db')
                commands.append(db_name)

            subprocess.run(commands)
            return True
        except Exception as exc:
            print('[restore_db] An error occurred while restoring {0}'.format(data_dir))
            print(str(exc))
            return False

    @staticmethod
    def get_data_collection_names(db_name):
        '''Returns the name of all black box data collections in the specified database.

        Keyword arguments:
        @param db_name -- name of a MongoDB database

        '''
        client = pm.MongoClient()
        db = client[db_name]
        collection_names = db.list_collection_names()
        filtered_collection_names = [collection for collection in collection_names
                                     if collection != 'system.indexes' and
                                     collection != 'black_box_metadata']
        return filtered_collection_names

    @staticmethod
    def clear_db(db_name):
        '''Drop all collections in the given database.

        Keyword arguments:
        @param db_name -- name of a MongoDB database

        '''
        client = pm.MongoClient()
        db = client[db_name]
        collections = DBUtils.get_data_collection_names(db_name)
        for collection in collections:
            db.drop_collection(collection)

    @staticmethod
    def get_all_docs(db_name, collection_name):
        '''Returns all documents contained in the specified collection of the given database

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection from which to take data

        '''
        return DBUtils.get_docs(db_name, collection_name)

    @staticmethod
    def get_docs(db_name, collecion_name, start_time=-1, stop_time=-1):
        """Returns all documents contained in the specific collection of the
        given database within given time duration

        @param db_name -- string (name of MongoDB database) 
        @param collecion_name -- string (name of a collection from that db)
        @param start_time -- float (start time stamp)
        @param stop_time -- float (stop time stamp)

        """
        doc_cursor = DBUtils.get_doc_cursor(db_name, collecion_name, start_time, stop_time)
        docs = [doc for doc in doc_cursor]
        return docs

    @staticmethod
    def get_docs_of_last_n_secs(db_name, collecion_name, n=3):
        """Return documents from given collection name of last n seconds from 
        given db name

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection from which to take data
        @param n -- float (duration in seconds)

        @returns list of dicts

        """
        stop_time = DBUtils.get_db_newest_timestamp(db_name)
        start_time = stop_time - n
        return DBUtils.get_docs(db_name, collecion_name, start_time, stop_time)

    @staticmethod
    def get_doc_cursor(db_name, collection_name, start_time=-1, stop_time=-1, port=27017):
        '''Returns a cursor for all documents in the specified collection of 
        the given database which have the 'timestamp' value in the given range

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection from which to take data
        @param start_time -- float (starting time stamp)
        @param stop_time -- float (stoping time stamp)

        '''
        client = pm.MongoClient(port=port)
        database = client[db_name]
        collection = database[collection_name]

        docs = {}
        if start_time == -1 and stop_time == -1:
            docs = collection.find({})
        elif start_time == -1:
            docs = collection.find({'timestamp': {'$lte': stop_time}})
        elif stop_time == -1:
            docs = collection.find({'timestamp': {'$gte': start_time}})
        else:
            docs = collection.find({'timestamp': {'$gte': start_time, '$lte': stop_time}})
        return docs

    @staticmethod
    def get_collection_metadata(db_name, collection_name):
        '''Returns the entry of the 'black_box_metadata' collection for the specified collection

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection whose metadata should be retrieved

        '''
        client = pm.MongoClient()
        db = client[db_name]
        collection = db['black_box_metadata']
        metadata_doc = collection.find_one({'collection_name': collection_name})
        return metadata_doc

    @staticmethod
    def get_oldest_doc(db_name, collection_name):
        '''Returns the oldest document in the given collection name

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection

        '''
        client = pm.MongoClient()
        db = client[db_name]
        collection = db[collection_name]
        doc = collection.find_one(sort=[('timestamp', pm.ASCENDING)])
        return doc

    @staticmethod
    def get_newest_doc(db_name, collection_name):
        '''Returns the newest document in the given collection name

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection

        '''
        client = pm.MongoClient()
        db = client[db_name]
        collection = db[collection_name]
        doc = collection.find_one(sort=[('timestamp', pm.DESCENDING)])
        return doc

    @staticmethod
    def get_db_oldest_timestamp(db_name):
        """get the oldest record in the mongo db and return the corresponding
        timestamp

        Keyword arguments:
        @param db_name -- name of a MongoDB database

        @returns: float

        """
        data_collections = DBUtils.get_data_collection_names(db_name)
        start_timestamp = float('inf')
        for collection in data_collections:
            oldest_doc = DBUtils.get_oldest_doc(db_name, collection)
            if oldest_doc['timestamp'] < start_timestamp:
                start_timestamp = oldest_doc['timestamp']
        return start_timestamp

    @staticmethod
    def get_db_newest_timestamp(db_name):
        """get the newest record in the mongo db and return the corresponding
        timestamp

        Keyword arguments:
        @param db_name -- name of a MongoDB database

        @returns: float

        """
        data_collections = DBUtils.get_data_collection_names(db_name)
        stop_timestamp = 0.0
        for collection in data_collections:
            newest_doc = DBUtils.get_newest_doc(db_name, collection)
            if newest_doc['timestamp'] > stop_timestamp:
                stop_timestamp = newest_doc['timestamp']
        return stop_timestamp
