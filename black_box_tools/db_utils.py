import os
from typing import Tuple, Sequence, Dict
import subprocess
import pymongo as pm

class DBUtils(object):
    @staticmethod
    def restore_subdbs(db_name: str,
                       data_dirs: Sequence[str],
                       subdb_names: Sequence[str]) -> bool:
        '''Restores multiple dumps into a single database. For each
        of the dumps, a metadata entry with the following contents
        is added to the in the "black_box_metadata" collection:
        {
            "type": "database",
            "name": subdb_names[i],
            "start_time": oldest timestamp in the database,
-           "end_time": latest timestamp in the database
        }

        Keyword arguments:
        db_name: str -- name of the database in which the data are restored
        data_dirs: Sequence[str] -- list of directory names with database dumps
        subdb_names: Sequence[str] -- list of names of the database dumps
                                      (semantic descriptions of the data)

        '''
        data_dir = None
        try:
            for i, data_dir in enumerate(data_dirs):
                print('[restore_db_list] Restoring data from {0}'.format(data_dir))

                # we restore the data into a temporary database and read the
                # oldest and newest data timestamp from it; the temporary
                # database is then removed
                DBUtils.restore_db(data_dir, drop_existing_records=True, db_name='restore_temp')
                oldest_timestamp = DBUtils.get_db_oldest_timestamp('restore_temp')
                newest_timestamp = DBUtils.get_db_newest_timestamp('restore_temp')
                DBUtils.drop_db('restore_temp')

                # we restore the data into the combined database
                DBUtils.restore_db(data_dir, drop_existing_records=False, db_name=db_name)

                # we add metadata about the imported database into the combined database
                client = DBUtils.get_db_client()
                db = client[db_name]
                metadata_collection = db['black_box_metadata']
                metadata = {'type': 'database',
                            'name': subdb_names[i],
                            'start_time': oldest_timestamp,
                            'end_time': newest_timestamp}
                metadata_collection.insert_one(metadata)
        except:
            print('[restore_db_list] An error occurred while restoring {0}'.format(data_dir))
            raise

    @staticmethod
    def restore_db(data_dir: str, drop_existing_records: bool=True,
                   db_name: str=None) -> bool:
        '''Restores a MongoDB database dumped in the given directory. If "db_name"
        is not passed or is set to None, the name of the restored database
        will be the same as the name of the directory. Returns True if
        everything goes well; raises an exception otherwise.

        Keyword arguments:
        @param data_dir: str -- absolute path of a directory containing a dumped MongoDB database
        @param drop_existing_records: bool -- indicates whether to drop any existing records
                                              if a database with the same name already
                                              exists (default True)
        @param db_name: str -- name of the database for restoring the data (default None,
                               in which case the database name is the same as the name
                               of the data directory)

        '''
        try:
            (host, port) = DBUtils.get_db_host_and_port()

            commands = ['mongorestore', data_dir]
            if drop_existing_records:
                commands.append('--drop')

            if not db_name:
                db_name = os.path.basename(data_dir)
            commands.extend(['--db', db_name, '--host', host, '--port', str(port)])

            with open(os.devnull, 'w') as devnull:
                process = subprocess.run(commands, stdout=devnull, stderr=devnull)
            return process.returncode == 0
        except:
            print('[restore_db] An error occurred while restoring {0}'.format(data_dir))
            raise

    @staticmethod
    def dump_db(db_name: str, data_dir: str='.', delete_db: bool=True) -> bool:
        '''Dumps a MongoDB database in the specified directory. If "delete_db"
        is set, deletes the database after dumping it.

        Keyword arguments:
        @param db_name: str -- name of the database to be dumped
        @param data_dir: str -- absolute path of a directory where the dump
                                should be created (default '.')
        @param delete_db: bool -- indicates whether to drop the database after
                                  dumping (default True)

        '''
        try:
            (host, port) = DBUtils.get_db_host_and_port()

            # we dump the database
            command = ['mongodump', '--db', db_name, '--out', data_dir,
                       '--host', host, '--port', str(port)]
            with open(os.devnull, 'w') as devnull:
                process = subprocess.run(command, stdout=devnull, stderr=devnull)

            if delete_db:
                client = DBUtils.get_db_client()
                client.drop_database(db_name)
            return process.returncode == 0
        except:
            print('[restore_db] An error occurred while restoring {0}'.format(data_dir))
            raise

    @staticmethod
    def get_data_collection_names(db_name: str) -> Sequence[str]:
        '''Returns the names of all black box data collections in the specified database.

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database

        '''
        client = DBUtils.get_db_client()
        db = client[db_name]
        collection_names = db.list_collection_names()
        filtered_collection_names = [collection for collection in collection_names
                                     if collection != 'system.indexes' and
                                     collection != 'black_box_metadata']
        return filtered_collection_names

    @staticmethod
    def clear_db(db_name: str) -> None:
        '''Drop all collections in the given database.

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database

        '''
        client = DBUtils.get_db_client()
        db = client[db_name]
        collections = DBUtils.get_data_collection_names(db_name)
        for collection in collections:
            db.drop_collection(collection)

    @staticmethod
    def drop_db(db_name: str) -> None:
        '''Drops the given database.

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database

        '''
        client = DBUtils.get_db_client()
        client.drop_database(db_name)

    @staticmethod
    def get_all_docs(db_name: str, collection_name: str) -> Sequence[Dict]:
        '''Returns all documents contained in the specified collection of the given database

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection from which to take data

        '''
        return DBUtils.get_docs(db_name, collection_name)

    @staticmethod
    def get_subdb_metadata(db_name: str) -> Sequence[Dict[str, str]]:
        '''Returns a list of dictionaries containing metadata about
        any sub-databases stored in the given database. In particular,
        the returned dictionaries are of the following form:
        {
            "type": "database",
            "name": name of a sub-database,
            "start_time": oldest timestamp in the database,
-           "end_time": latest timestamp in the database
        }

        Keyword arguments:
        db_name: str -- name of a MongoDB database

        '''
        client = DBUtils.get_db_client()
        db = client[db_name]
        collection = db['black_box_metadata']
        db_data = [x for x in collection.find({'type': 'database'})]
        return db_data

    @staticmethod
    def get_subdb_docs(db_name: str, collection_name: str,
                       subdb_metadata: Sequence[Dict[str, str]]) -> Sequence[Dict]:
        '''Returns a list of dictionaries in which the keys are sub-database
        names and the values are lists of documents in the given collection
        corresponding to the respective sub-databases. The documents belonging
        to a particular sub-database are obtained based on the sub-database timestamps,
        namely the assumption is that the sub-databases do not overlap temporally.

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection from which to take data
        @param subdb_metadata: Sequence[Dict[str, str]]: a list of dictionaries as
                                                         returned by get_subdb_metadata

        '''
        docs = {}
        for subdb in subdb_metadata:
            docs[subdb['name']] = DBUtils.get_docs(db_name, collection_name,
                                                   start_time=subdb['start_time'],
                                                   stop_time=subdb['end_time'])
        return docs

    @staticmethod
    def get_docs(db_name: str, collection_name: str,
                 start_time: float=-1, stop_time: float=-1) -> Sequence[Dict]:
        """Returns all documents contained in the specific collection of the
        given database within given time duration

        @param db_name -- string (name of MongoDB database)
        @param collection_name -- string (name of a collection from that db)
        @param start_time -- float (start time stamp)
        @param stop_time -- float (stop time stamp)

        """
        doc_cursor = DBUtils.get_doc_cursor(db_name, collection_name, start_time, stop_time)
        docs = [doc for doc in doc_cursor]
        return docs

    @staticmethod
    def get_docs_of_last_n_secs(db_name: str, collection_name: str,
                                n: float=3.) -> Sequence[Dict]:
        """Return documents from given collection name of last n seconds from
        given db name

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection from which to take data
        @param n: float -- duration in seconds (default 3.)

        """
        stop_time = DBUtils.get_db_newest_timestamp(db_name)
        start_time = stop_time - n
        return DBUtils.get_docs(db_name, collection_name, start_time, stop_time)

    @staticmethod
    def get_doc_cursor(db_name: str, collection_name: str,
                       start_time: float=-1., stop_time: float=-1) -> pm.cursor.Cursor:
        '''Returns a cursor for all documents in the specified collection of
        the given database which have the 'timestamp' value in the given range.
        If both "start_time" and "end_time" are -1, returns all documents. If
        only "start_time" is -1, returns all documents up to "end_time". If only
        "end_time" is -1, returns all documents starting at "start_time".

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection from which to take data
        @param start_time: float -- starting time stamp (default -1.)
        @param stop_time: float -- stoping time stamp (default -1.)

        '''
        client = DBUtils.get_db_client()
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
    def get_collection_metadata(db_name: str, collection_name: str) -> Dict:
        '''Returns the entry of the 'black_box_metadata' collection for the specified collection

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection whose metadata should be retrieved

        '''
        client = DBUtils.get_db_client()
        db = client[db_name]
        collection = db['black_box_metadata']
        metadata_doc = collection.find_one({'collection_name': collection_name})
        return metadata_doc

    @staticmethod
    def get_oldest_doc(db_name: str, collection_name: str) -> Dict:
        '''Returns the oldest document in the given collection name

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection

        '''
        client = DBUtils.get_db_client()
        db = client[db_name]
        collection = db[collection_name]
        doc = collection.find_one(sort=[('timestamp', pm.ASCENDING)])
        return doc

    @staticmethod
    def get_newest_doc(db_name: str, collection_name: str) -> Dict:
        '''Returns the newest document in the given collection name

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection

        '''
        client = DBUtils.get_db_client()
        db = client[db_name]
        collection = db[collection_name]
        doc = collection.find_one(sort=[('timestamp', pm.DESCENDING)])
        return doc

    @staticmethod
    def get_last_doc_before(db_name: str, collection_name: str,
                            timestamp_filter: float=-1.):
        '''Returns the last document in the collection with the given name
        that is before given timestamp. If timestamp has a negative value,
        the most recent document in the collection is returned.

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database
        @param collection_name: str -- name of a collection
        @param timestamp_filter: float -- latest time of the last record in
                                          the given collection (default -1.)

        '''
        if timestamp_filter < 0:
            return DBUtils.get_newest_doc(db_name, collection_name)

        client = DBUtils.get_db_client()
        database = client[db_name]
        collection = database[collection_name]
        return collection.find_one({'timestamp': {'$lte': timestamp_filter}},
                                   sort=[('timestamp', pm.DESCENDING)])

    @staticmethod
    def get_db_oldest_timestamp(db_name: str) -> float:
        """get the oldest record in the mongo db and return the corresponding
        timestamp

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database

        """
        data_collections = DBUtils.get_data_collection_names(db_name)
        start_timestamp = float('inf')
        for collection in data_collections:
            oldest_doc = DBUtils.get_oldest_doc(db_name, collection)
            if oldest_doc['timestamp'] < start_timestamp:
                start_timestamp = oldest_doc['timestamp']
        return start_timestamp

    @staticmethod
    def get_db_newest_timestamp(db_name: str) -> float:
        """get the newest record in the mongo db and return the corresponding
        timestamp

        Keyword arguments:
        @param db_name: str -- name of a MongoDB database

        """
        data_collections = DBUtils.get_data_collection_names(db_name)
        stop_timestamp = 0.0
        for collection in data_collections:
            newest_doc = DBUtils.get_newest_doc(db_name, collection)
            if newest_doc['timestamp'] > stop_timestamp:
                stop_timestamp = newest_doc['timestamp']
        return stop_timestamp

    @staticmethod
    def get_db_client() -> pm.MongoClient:
        '''Returns a MongoDB client at <host>:<port>. By default,
        <host> is "localhost" and <port> is 27017, but these values can
        be overriden by setting the environment variables "DB_HOST" and
        "DB_PORT" respectively.
        '''
        (host, port) = DBUtils.get_db_host_and_port()
        client = pm.MongoClient(host=host, port=port)
        return client

    @staticmethod
    def get_db_host_and_port() -> Tuple[str, int]:
        '''Returns a (host, port) tuple which is ("localhost", 27017) by default,
        but the values can be overridden by setting the environment variables
        "DB_HOST" and "DB_PORT" respectively.
        '''
        host = 'localhost'
        port = 27017
        if 'DB_HOST' in os.environ:
            host = os.environ['DB_HOST']
        if 'DB_PORT' in os.environ:
            port = int(os.environ['DB_PORT'])
        return (host, port)
