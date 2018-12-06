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
    def get_all_docs(db_name, collection_name):
        '''Returns all documents contained in the specified collection of the given database

        Keyword arguments:
        @param db_name -- name of a MongoDB database
        @param collection_name -- name of a collection from which to take data

        '''
        client = pm.MongoClient()
        db = client[db_name]
        collection = db[collection_name]
        doc_cursor = collection.find({})
        docs = [doc for doc in doc_cursor]
        return docs
